package edu.wpi.team190.gompeilib.robot.mechanisms.flywheel;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.io.motor.MotorControllerGroup;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.physics.FlywheelMechanismSim;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.math.CustomUnits;
import edu.wpi.team190.gompeilib.robot.mechanisms.ConstraintConfigurable;
import edu.wpi.team190.gompeilib.robot.mechanisms.CurrentControl;
import edu.wpi.team190.gompeilib.robot.mechanisms.CurrentControllable;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismConversion;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismCore;
import edu.wpi.team190.gompeilib.robot.mechanisms.MotionReadable;
import edu.wpi.team190.gompeilib.robot.mechanisms.TuningBindings;
import edu.wpi.team190.gompeilib.robot.mechanisms.VelocityControl;
import edu.wpi.team190.gompeilib.robot.mechanisms.VelocityControllable;
import edu.wpi.team190.gompeilib.robot.mechanisms.VoltageControl;
import edu.wpi.team190.gompeilib.robot.mechanisms.VoltageControllable;
import edu.wpi.team190.gompeilib.sysid.CustomSysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularAccelerationUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.VoltageUnit;

/**
 * A velocity-controlled flywheel: one leader motor and any number of followers. Voltage velocity
 * control uses gain slot {@code ZERO}; torque-current velocity control uses slot {@code ONE}.
 */
public class GenericFlywheel
    implements VelocityControllable<AngularVelocityUnit>,
        VoltageControllable,
        CurrentControllable,
        MotionReadable<AngleUnit, AngularVelocityUnit>,
        ConstraintConfigurable<AngularVelocityConstraints> {
  private static final MechanismConversion<AngleUnit, AngularVelocityUnit, AngularAccelerationUnit>
      CONVERSION = MechanismConversion.angular();

  private final MechanismCore core;
  private final VelocityControl<AngularVelocityUnit> velocityControl;
  private final VoltageControl voltageControl;
  private final CurrentControl currentControl;

  private final CustomSysIdRoutine<VoltageUnit> voltageCharacterizationRoutine;
  private final CustomSysIdRoutine<CurrentUnit> torqueCharacterizationRoutine;

  /**
   * Creates a flywheel. It starts idle.
   *
   * @param motorFactory how to build the flywheel's motors
   * @param subsystem the subsystem that owns the flywheel, for its log prefix
   * @param constants the flywheel's constants
   * @param name the flywheel's name; it logs under {@code <subsystem>/<name>}
   * @param velocityGoal the initial velocity goal
   * @param voltageGoal the initial voltage goal
   */
  public GenericFlywheel(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      GenericFlywheelConstants constants,
      String name,
      Setpoint<AngularVelocityUnit> velocityGoal,
      Setpoint<VoltageUnit> voltageGoal) {
    MotorControllerGroup motors =
        new MotorControllerGroup(
            constants.motor.toBuilder().withMotionProfile(profile(constants.constraints)).build(),
            motorFactory);
    core = new MechanismCore(motors, subsystem, name);
    boolean foc = constants.motor.enableFOC;
    velocityControl =
        new VelocityControl<>(
            core,
            CONVERSION,
            velocityGoal,
            () -> constants.constraints.goalTolerance().get(),
            foc,
            GainSlot.ZERO,
            GainSlot.ONE);
    voltageControl = new VoltageControl(core, voltageGoal, foc);
    currentControl =
        new CurrentControl(core, new Setpoint<>(Amps.of(0), constants.currentOffsetStep));

    String topic = core.getTopic();
    voltageCharacterizationRoutine =
        new CustomSysIdRoutine<>(
            new CustomSysIdRoutine.Config<VoltageUnit>(
                CustomUnits.voltsPerSecond.ofNative(0.5),
                Volts.of(8),
                Seconds.of(24),
                (state) -> Logger.recordOutput(topic + "/Voltage SysID State", state.toString()),
                Volts),
            new CustomSysIdRoutine.Mechanism<>(
                (volts) ->
                    core.setControl(
                        new OpenLoopRequest.VoltageOutput(
                            Volts.of(volts.in(Volts)), foc, RequestOptions.DEFAULT)),
                subsystem));
    torqueCharacterizationRoutine =
        new CustomSysIdRoutine<>(
            new CustomSysIdRoutine.Config<CurrentUnit>(
                CustomUnits.ampsPerSecond.ofNative(5.0),
                Amps.of(40),
                Seconds.of(10),
                (state) ->
                    Logger.recordOutput(topic + "/Torque Current SysID State", state.toString()),
                Amp),
            new CustomSysIdRoutine.Mechanism<>(
                (amps) ->
                    core.setControl(new OpenLoopRequest.CurrentOutput(Amps.of(amps.in(Amps)))),
                subsystem));

    TuningBindings.bindGains(motors, constants.motor);
    TuningBindings.bindConstraints(motors, constants.constraints, GenericFlywheel::profile);
  }

  /**
   * Creates a flywheel with zero velocity and voltage goals.
   *
   * @param motorFactory how to build the flywheel's motors
   * @param subsystem the subsystem that owns the flywheel, for its log prefix
   * @param constants the flywheel's constants
   * @param name the flywheel's name; it logs under {@code <subsystem>/<name>}
   */
  public GenericFlywheel(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      GenericFlywheelConstants constants,
      String name) {
    this(
        motorFactory,
        subsystem,
        constants,
        name,
        new Setpoint<>(
            RadiansPerSecond.of(0),
            constants.velocityOffsetStep,
            RadiansPerSecond.of(-constants.motor.gearRatio * constants.flywheelGearbox.freeSpeed),
            RadiansPerSecond.of(constants.motor.gearRatio * constants.flywheelGearbox.freeSpeed)),
        new Setpoint<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12)));
  }

  /**
   * Creates a flywheel with a zero voltage goal.
   *
   * @param motorFactory how to build the flywheel's motors
   * @param subsystem the subsystem that owns the flywheel, for its log prefix
   * @param constants the flywheel's constants
   * @param name the flywheel's name; it logs under {@code <subsystem>/<name>}
   * @param velocityGoal the initial velocity goal
   */
  public GenericFlywheel(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      GenericFlywheelConstants constants,
      String name,
      Setpoint<AngularVelocityUnit> velocityGoal) {
    this(
        motorFactory,
        subsystem,
        constants,
        name,
        velocityGoal,
        new Setpoint<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12)));
  }

  /**
   * Builds the physics model for simulating a flywheel.
   *
   * @param constants the flywheel's constants
   * @return the physics model
   */
  public static MechanismSim simMechanism(GenericFlywheelConstants constants) {
    return new FlywheelMechanismSim(
        constants.flywheelGearbox, constants.momentOfInertia, constants.motor.gearRatio);
  }

  private static MotionProfileConfig profile(AngularVelocityConstraints constraints) {
    return CONVERSION.trapezoidal(
        constraints.maxVelocity().get(), constraints.maxAcceleration().get());
  }

  @Override
  public MechanismCore core() {
    return core;
  }

  @Override
  public VelocityControl<AngularVelocityUnit> velocityControl() {
    return velocityControl;
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
  public MechanismConversion<AngleUnit, AngularVelocityUnit, AngularAccelerationUnit> conversion() {
    return CONVERSION;
  }

  @Override
  public MotionProfileConfig motionProfile(AngularVelocityConstraints constraints) {
    return profile(constraints);
  }

  @Override
  @Trace
  public void periodic() {
    core.periodic();
  }

  public Command sysIdRoutineVoltage() {
    return Commands.sequence(
        Commands.runOnce(core::idle),
        voltageCharacterizationRoutine
            .dynamic(CustomSysIdRoutine.Direction.kForward)
            .withTimeout(10),
        Commands.waitSeconds(6.0),
        voltageCharacterizationRoutine
            .dynamic(CustomSysIdRoutine.Direction.kReverse)
            .withTimeout(10),
        Commands.waitSeconds(6.0),
        voltageCharacterizationRoutine.quasistatic(CustomSysIdRoutine.Direction.kForward),
        Commands.waitSeconds(6.0),
        voltageCharacterizationRoutine.quasistatic(CustomSysIdRoutine.Direction.kReverse));
  }

  public Command sysIdRoutineTorque() {
    return Commands.sequence(
        Commands.runOnce(core::idle),
        torqueCharacterizationRoutine.dynamic(CustomSysIdRoutine.Direction.kForward),
        Commands.waitSeconds(20.0),
        torqueCharacterizationRoutine.dynamic(CustomSysIdRoutine.Direction.kReverse),
        Commands.waitSeconds(20.0),
        torqueCharacterizationRoutine.quasistatic(CustomSysIdRoutine.Direction.kForward),
        Commands.waitSeconds(20.0),
        torqueCharacterizationRoutine.quasistatic(CustomSysIdRoutine.Direction.kReverse));
  }
}
