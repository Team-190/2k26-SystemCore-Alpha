package edu.wpi.team190.gompeilib.robot.mechanisms.flywheel;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.io.motor.MotorControllerGroup;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.physics.FlywheelMechanismSim;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.mechanisms.Characterizable;
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
import edu.wpi.team190.gompeilib.sysid.SysIdConfig;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularAccelerationUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.VoltageUnit;

/**
 * A velocity-controlled flywheel: one leader motor and any number of followers. Voltage velocity
 * control uses its voltage gains in slot {@code ZERO}; torque-current velocity control uses its
 * current gains in slot {@code ONE}.
 */
public class GenericFlywheel
    implements VelocityControllable<AngularVelocityUnit>,
        VoltageControllable,
        CurrentControllable,
        MotionReadable<AngleUnit, AngularVelocityUnit>,
        ConstraintConfigurable<AngularVelocityConstraints>,
        Characterizable {
  /** The slot voltage velocity control uses. */
  public static final GainSlot VOLTAGE_SLOT = GainSlot.ZERO;

  /** The slot torque-current velocity control uses. */
  public static final GainSlot CURRENT_SLOT = GainSlot.ONE;

  private static final MechanismConversion<AngleUnit, AngularVelocityUnit, AngularAccelerationUnit>
      CONVERSION = MechanismConversion.angular();

  private final MechanismCore core;
  private final VelocityControl<AngularVelocityUnit> velocityControl;
  private final VoltageControl voltageControl;
  private final CurrentControl currentControl;

  private final SysIdConfig<VoltageUnit> voltageSysIdConfig;
  private final SysIdConfig<CurrentUnit> currentSysIdConfig;

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
    if (!constants.motor.slots.isEmpty()) {
      throw new IllegalArgumentException(
          name + "'s motor slots must be empty; set voltageGains and currentGains instead");
    }
    MotorConfig config =
        constants.motor.toBuilder()
            .withMotionProfile(profile(constants.constraints))
            .withSlot(VOLTAGE_SLOT, SlotConfig.velocity(constants.voltageGains))
            .withSlot(CURRENT_SLOT, SlotConfig.velocity(constants.currentGains))
            .build();
    MotorControllerGroup motors = new MotorControllerGroup(config, motorFactory);
    core = new MechanismCore(motors, subsystem, name);
    boolean foc = constants.motor.enableFOC;
    velocityControl =
        new VelocityControl<>(
            core,
            CONVERSION,
            velocityGoal,
            () -> constants.constraints.goalTolerance().get(),
            foc,
            VOLTAGE_SLOT,
            CURRENT_SLOT);
    voltageControl = new VoltageControl(core, voltageGoal, foc);
    currentControl =
        new CurrentControl(core, new Setpoint<>(Amps.of(0), constants.currentOffsetStep));

    voltageSysIdConfig = constants.voltageSysId;
    currentSysIdConfig = constants.currentSysId;

    TuningBindings.bindGains(motors, config);
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

  /**
   * Replaces the voltage velocity gains. The gains are not watched for edits.
   *
   * @param gains the gains
   */
  public void updateVoltageGains(
      Gains<VoltageUnit, AngularVelocityUnit, AngularVelocityUnit> gains) {
    core.getMotors().updateGains(SlotConfig.velocity(gains), VOLTAGE_SLOT);
  }

  /**
   * Replaces the torque-current velocity gains. The gains are not watched for edits.
   *
   * @param gains the gains
   */
  public void updateCurrentGains(
      Gains<CurrentUnit, AngularVelocityUnit, AngularVelocityUnit> gains) {
    core.getMotors().updateGains(SlotConfig.velocity(gains), CURRENT_SLOT);
  }

  /**
   * Replaces a slot's gains, checking that the voltage and torque-current slots get gains in their
   * own output type.
   *
   * @param gains the slot's gains
   * @param slot the slot
   * @throws IllegalArgumentException if the gains' output doesn't match the slot's requests
   */
  @Override
  public void updateGains(SlotConfig gains, GainSlot slot) {
    if (slot == VOLTAGE_SLOT) {
      SlotConfig.requireOutputType(gains, OutputType.VOLTAGE, core.getTopic() + " voltage slot");
    } else if (slot == CURRENT_SLOT) {
      SlotConfig.requireOutputType(gains, OutputType.CURRENT, core.getTopic() + " current slot");
    }
    core.getMotors().updateGains(gains, slot);
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

  /**
   * Returns a command that characterizes the flywheel in voltage.
   *
   * @return the command
   */
  public Command sysIdRoutineVoltage() {
    return characterize(voltageSysIdConfig, voltageControl::request);
  }

  /**
   * Returns a command that characterizes the flywheel in torque current.
   *
   * @return the command
   */
  public Command sysIdRoutineTorque() {
    return characterize(currentSysIdConfig, currentControl::request);
  }
}
