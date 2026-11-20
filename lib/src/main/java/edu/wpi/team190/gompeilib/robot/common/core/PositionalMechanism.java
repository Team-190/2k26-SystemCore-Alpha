package edu.wpi.team190.gompeilib.robot.common.core;

import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConstraint;
import edu.wpi.team190.gompeilib.characterization.CharacterizationRoutine;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.Constraints;
import edu.wpi.team190.gompeilib.io.motor.MotorControllerGroup;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.robot.common.characterization.Characterizable;
import edu.wpi.team190.gompeilib.robot.common.characterization.ConstraintConfigurable;
import edu.wpi.team190.gompeilib.robot.common.homing.CurrentHoming;
import edu.wpi.team190.gompeilib.robot.common.homing.Homeable;
import edu.wpi.team190.gompeilib.robot.common.openloop.CurrentControllable;
import edu.wpi.team190.gompeilib.robot.common.openloop.DutyCycleControllable;
import edu.wpi.team190.gompeilib.robot.common.openloop.OpenLoopControl;
import edu.wpi.team190.gompeilib.robot.common.openloop.OpenLoopControls;
import edu.wpi.team190.gompeilib.robot.common.openloop.VoltageControllable;
import edu.wpi.team190.gompeilib.robot.common.position.MotionReadable;
import edu.wpi.team190.gompeilib.robot.common.position.PositionControl;
import edu.wpi.team190.gompeilib.robot.common.position.PositionControllable;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.DimensionlessUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Dimensionless;

/**
 * A position-controlled mechanism over a travel range: one leader motor and any number of
 * followers, closed-loop position control in the mechanism's units through its conversion, the
 * open-loop voltage, current, and duty-cycle controls, optional current homing against a hard stop,
 * and characterization that stops short of the travel limits. It starts idle.
 *
 * <p>A subclass supplies the motor config, conversion, and limits, implements {@link
 * #motionProfile}, and overrides {@link #periodic()} with {@code @Trace} so it is timed under its
 * own name.
 *
 * @param <P> the mechanism's position unit
 * @param <V> the mechanism's velocity unit
 * @param <A> the mechanism's acceleration unit
 * @param <C> the mechanism's constraints type
 */
public abstract class PositionalMechanism<
        P extends Unit, V extends Unit, A extends Unit, C extends Constraints<C>>
    implements PositionControllable<P>,
        VoltageControllable,
        CurrentControllable,
        DutyCycleControllable,
        MotionReadable<P, V>,
        ConstraintConfigurable<C>,
        Homeable,
        Characterizable {
  private final MechanismCore core;
  private final MechanismConversion<P, V, A> conversion;
  private final PositionControl<P> positionControl;
  private final OpenLoopControls openLoop;
  private final Optional<CurrentHoming<P>> homing;
  private final CharacterizationConfig<?> characterizationConfig;
  private final CharacterizationConstraint<P> characterizationConstraint;

  /**
   * Builds the mechanism's motors and controls, and sends gain and constraint edits to the motors.
   *
   * @param motorFactory how to build the motors
   * @param motor the leader's config, with its slots, soft limits, and motion profile
   * @param subsystem the subsystem that owns the mechanism, for its log prefix
   * @param name the mechanism's name; it logs under {@code <subsystem>/<name>}
   * @param conversion the conversion between the mechanism's units and the gearbox output
   * @param constraints the tunable constraints, whose edits rebuild the motion profile
   * @param goalTolerance how close the position must be to count as at the goal
   * @param minPosition the lowest position
   * @param maxPosition the highest position
   * @param characterizationMargin how far short of the limits a characterization leg stops
   * @param closedLoopOutputType the output position control drives
   * @param positionGoal the initial position goal
   * @param voltageGoal the initial voltage goal
   * @param currentStep how far one increment or decrement moves the current goal
   * @param dutyCycleStep how far one increment or decrement moves the duty-cycle goal
   * @param homing how the mechanism homes, or empty if it doesn't
   * @param characterization how the mechanism is characterized
   */
  protected PositionalMechanism(
      MotorIOFactory motorFactory,
      MotorConfig motor,
      Subsystem subsystem,
      String name,
      MechanismConversion<P, V, A> conversion,
      C constraints,
      Supplier<? extends Measure<P>> goalTolerance,
      Measure<P> minPosition,
      Measure<P> maxPosition,
      Supplier<? extends Measure<P>> characterizationMargin,
      OutputType closedLoopOutputType,
      Setpoint<P> positionGoal,
      Setpoint<VoltageUnit> voltageGoal,
      Current currentStep,
      Dimensionless dutyCycleStep,
      Optional<CurrentHoming.Config<P>> homing,
      CharacterizationConfig<?> characterization) {
    this.conversion = Objects.requireNonNull(conversion, "conversion");
    MotorControllerGroup motors = new MotorControllerGroup(motor, motorFactory);
    core = new MechanismCore(motors, subsystem, name);
    positionControl =
        new PositionControl<>(
            core, conversion, positionGoal, goalTolerance, motor.enableFOC, closedLoopOutputType);
    openLoop = OpenLoopControls.create(core, motor, voltageGoal, currentStep, dutyCycleStep);
    this.homing =
        Objects.requireNonNull(homing, "homing")
            .map(
                config ->
                    new CurrentHoming<>(
                        core, positionControl, openLoop.voltage(), openLoop.current(), config));
    characterizationConfig = Objects.requireNonNull(characterization, "characterization");
    characterizationConstraint =
        new CharacterizationConstraint<>(
            this::getPosition, minPosition, maxPosition, characterizationMargin);

    TuningBindings.bindGains(motors, motor);
    TuningBindings.bindConstraints(motors, constraints, this::motionProfile);
  }

  @Override
  public MechanismCore core() {
    return core;
  }

  @Override
  public MechanismConversion<P, V, A> conversion() {
    return conversion;
  }

  @Override
  public PositionControl<P> positionControl() {
    return positionControl;
  }

  @Override
  public OpenLoopControl<VoltageUnit> voltageControl() {
    return openLoop.voltage();
  }

  @Override
  public OpenLoopControl<CurrentUnit> currentControl() {
    return openLoop.current();
  }

  @Override
  public OpenLoopControl<DimensionlessUnit> dutyCycleControl() {
    return openLoop.dutyCycle();
  }

  @Override
  public CurrentHoming<P> homing() {
    return homing.orElseThrow(
        () -> new IllegalStateException(core.getTopic() + " has no homing config"));
  }

  @Override
  public Optional<CharacterizationConstraint<?>> characterizationConstraint() {
    return Optional.of(characterizationConstraint);
  }

  /**
   * Returns this mechanism as a target driven open loop in a config's unit, for characterizing
   * several mechanisms together.
   *
   * @param config the config whose unit picks voltage or torque current
   * @param <U> the output unit
   * @return the target
   */
  public <U extends Unit> CharacterizationRoutine.Target<U> characterizationTarget(
      CharacterizationConfig<U> config) {
    return characterizationTarget(config, openLoop.voltage(), openLoop.current());
  }

  /**
   * Returns a command that characterizes the mechanism in its config's unit, stopping each leg
   * short of its travel limits, and prints its plant and gains.
   *
   * @return the command
   */
  public Command characterize() {
    return characterize(characterizationConfig, openLoop.voltage(), openLoop.current());
  }
}
