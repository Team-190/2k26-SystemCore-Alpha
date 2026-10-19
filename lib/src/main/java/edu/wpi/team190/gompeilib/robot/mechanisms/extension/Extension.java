package edu.wpi.team190.gompeilib.robot.mechanisms.extension;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.io.motor.MotorControllerGroup;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.io.motor.config.SoftLimits;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.physics.ElevatorMechanismSim;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.mechanisms.Characterizable;
import edu.wpi.team190.gompeilib.robot.mechanisms.ConstraintConfigurable;
import edu.wpi.team190.gompeilib.robot.mechanisms.CurrentControl;
import edu.wpi.team190.gompeilib.robot.mechanisms.CurrentControllable;
import edu.wpi.team190.gompeilib.robot.mechanisms.CurrentHoming;
import edu.wpi.team190.gompeilib.robot.mechanisms.DutyCycleControl;
import edu.wpi.team190.gompeilib.robot.mechanisms.DutyCycleControllable;
import edu.wpi.team190.gompeilib.robot.mechanisms.Homeable;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismConversion;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismCore;
import edu.wpi.team190.gompeilib.robot.mechanisms.MotionReadable;
import edu.wpi.team190.gompeilib.robot.mechanisms.PositionControl;
import edu.wpi.team190.gompeilib.robot.mechanisms.PositionControllable;
import edu.wpi.team190.gompeilib.robot.mechanisms.TuningBindings;
import edu.wpi.team190.gompeilib.robot.mechanisms.VoltageControl;
import edu.wpi.team190.gompeilib.robot.mechanisms.VoltageControllable;
import edu.wpi.team190.gompeilib.sysid.CustomSysIdRoutine;
import edu.wpi.team190.gompeilib.sysid.SysIdConstraint;
import java.util.Optional;
import lombok.Getter;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.LinearAccelerationUnit;
import org.wpilib.units.LinearVelocityUnit;
import org.wpilib.units.VoltageUnit;

/**
 * A linear extension driven through a drum: one leader motor and any number of followers. The
 * motors work in drum rotations; the extension's drum conversion turns travel into rotation and
 * back. It starts idle.
 */
public class Extension
    implements PositionControllable<DistanceUnit>,
        VoltageControllable,
        CurrentControllable,
        DutyCycleControllable,
        MotionReadable<DistanceUnit, LinearVelocityUnit>,
        ConstraintConfigurable<LinearConstraints>,
        Homeable,
        Characterizable {
  @Getter private final ExtensionConstants constants;
  private final MechanismConversion<DistanceUnit, LinearVelocityUnit, LinearAccelerationUnit>
      conversion;
  private final MechanismCore core;
  private final PositionControl<DistanceUnit> positionControl;
  private final VoltageControl voltageControl;
  private final CurrentControl currentControl;
  private final DutyCycleControl dutyCycleControl;
  private final Optional<CurrentHoming<DistanceUnit>> homing;

  private final SysIdConstraint<DistanceUnit> sysIdConstraint;

  /**
   * Creates an extension.
   *
   * @param motorFactory how to build the extension's motors
   * @param subsystem the subsystem that owns the extension, for its log prefix
   * @param name the extension's name; it logs under {@code <subsystem>/<name>}
   * @param constants the extension's constants
   * @param positionGoal the initial position goal
   * @param voltageGoal the initial voltage goal
   */
  public Extension(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      String name,
      ExtensionConstants constants,
      Setpoint<DistanceUnit> positionGoal,
      Setpoint<VoltageUnit> voltageGoal) {
    this.constants = constants;
    conversion = MechanismConversion.drum(constants.drumRadius);
    ExtensionConstants.ExtensionParameters parameters = constants.extensionParameters;
    if (!constants.motor.slots.isEmpty()) {
      throw new IllegalArgumentException(
          name + "'s motor slots must be empty; set its gains per distance in slots instead");
    }
    MotorConfig.MotorConfigBuilder motorBuilder =
        constants.motor.toBuilder()
            .withSoftLimits(
                SoftLimits.between(
                    conversion.toMotorPosition(parameters.minLength()),
                    conversion.toMotorPosition(parameters.maxLength())))
            .withMotionProfile(motionProfile(constants.constraints));
    constants.slots.forEach(
        (slot, gains) ->
            motorBuilder.withSlot(
                slot, SlotConfig.position(conversion.toMotorPositionGains(gains))));
    MotorConfig motor = motorBuilder.build();
    MotorControllerGroup motors = new MotorControllerGroup(motor, motorFactory);
    core = new MechanismCore(motors, subsystem, name);
    boolean foc = constants.motor.enableFOC;
    positionControl =
        new PositionControl<>(
            core,
            conversion,
            positionGoal,
            () -> constants.constraints.goalTolerance().get(),
            foc,
            constants.closedLoopOutputType);
    voltageControl = new VoltageControl(core, voltageGoal, foc);
    currentControl =
        new CurrentControl(core, new Setpoint<>(Amps.of(0), constants.currentOffsetStep));
    dutyCycleControl =
        new DutyCycleControl(
            core,
            new Setpoint<>(Value.of(0), constants.dutyCycleOffsetStep, Value.of(-1), Value.of(1)),
            foc);
    homing =
        constants.homing.map(config -> new CurrentHoming<>(core, positionControl, config, foc));

    sysIdConstraint =
        new SysIdConstraint<>(
            this::getPosition,
            parameters.minLength(),
            parameters.maxLength(),
            constants.constraints.sysIdMargin());

    TuningBindings.bindGains(motors, motor);
    TuningBindings.bindConstraints(motors, constants.constraints, this::motionProfile);
  }

  /**
   * Creates an extension with a position goal at zero, limited to its travel, and a 0 V goal.
   *
   * @param motorFactory how to build the extension's motors
   * @param subsystem the subsystem that owns the extension, for its log prefix
   * @param name the extension's name; it logs under {@code <subsystem>/<name>}
   * @param constants the extension's constants
   */
  public Extension(
      MotorIOFactory motorFactory, Subsystem subsystem, String name, ExtensionConstants constants) {
    this(
        motorFactory,
        subsystem,
        name,
        constants,
        new Setpoint<>(
            Meters.of(0),
            constants.heightOffsetStep,
            constants.extensionParameters.minLength(),
            constants.extensionParameters.maxLength()));
  }

  /**
   * Creates an extension with a 0 V goal.
   *
   * @param motorFactory how to build the extension's motors
   * @param subsystem the subsystem that owns the extension, for its log prefix
   * @param name the extension's name; it logs under {@code <subsystem>/<name>}
   * @param constants the extension's constants
   * @param positionGoal the initial position goal
   */
  public Extension(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      String name,
      ExtensionConstants constants,
      Setpoint<DistanceUnit> positionGoal) {
    this(
        motorFactory,
        subsystem,
        name,
        constants,
        positionGoal,
        new Setpoint<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12)));
  }

  /**
   * Builds the physics model for simulating an extension. It starts at the shortest extension.
   *
   * @param constants the extension's constants
   * @return the physics model
   */
  public static MechanismSim simMechanism(ExtensionConstants constants) {
    ExtensionConstants.ExtensionParameters parameters = constants.extensionParameters;
    return new ElevatorMechanismSim(
        parameters.carriageGearbox(),
        constants.motor.gearRatio,
        parameters.carriageMass(),
        constants.drumRadius,
        parameters.minLength(),
        parameters.maxLength(),
        parameters.simulateGravity(),
        parameters.minLength());
  }

  @Override
  public MechanismCore core() {
    return core;
  }

  @Override
  public PositionControl<DistanceUnit> positionControl() {
    return positionControl;
  }

  @Override
  public VoltageControl voltageControl() {
    return voltageControl;
  }

  @Override
  public CurrentControl currentControl() {
    return currentControl;
  }

  @Override
  public DutyCycleControl dutyCycleControl() {
    return dutyCycleControl;
  }

  @Override
  public CurrentHoming<DistanceUnit> homing() {
    return homing.orElseThrow(
        () -> new IllegalStateException(core.getTopic() + " has no homing config"));
  }

  @Override
  public MechanismConversion<DistanceUnit, LinearVelocityUnit, LinearAccelerationUnit>
      conversion() {
    return conversion;
  }

  @Override
  public MotionProfileConfig motionProfile(LinearConstraints constraints) {
    return conversion.trapezoidal(
        constraints.maxVelocity().get(), constraints.maxAcceleration().get());
  }

  @Override
  @Trace
  public void periodic() {
    core.periodic();
  }

  @Override
  public Optional<SysIdConstraint<?>> sysIdConstraint() {
    return Optional.of(sysIdConstraint);
  }

  /**
   * Returns this extension as a voltage SysId target, for characterizing several extensions
   * together.
   *
   * @return the target
   */
  public CustomSysIdRoutine.Target<VoltageUnit> voltageSysIdTarget() {
    return sysIdTarget(voltageControl::request);
  }

  /**
   * Returns a command that characterizes the extension in voltage, stopping each leg short of its
   * travel limits.
   *
   * @return the command
   */
  public Command runSysIdRoutine() {
    return characterize(constants.sysId, voltageControl::request);
  }
}
