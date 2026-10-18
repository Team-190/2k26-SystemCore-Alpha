package edu.wpi.team190.gompeilib.subsystems.flywheel;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.io.motor.MotorControllerGroup;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.NeutralRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.motor.outputs.VelocityRequest;
import edu.wpi.team190.gompeilib.io.physics.FlywheelMechanismSim;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.math.CustomUnits;
import edu.wpi.team190.gompeilib.sysid.CustomSysIdRoutine;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.units.*;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

/**
 * A velocity-controlled flywheel: one leader motor and any number of followers. Voltage velocity
 * control uses gain slot {@code ZERO}; torque-current velocity control uses slot {@code ONE}.
 */
public class GenericFlywheel {
  private static final Voltage VOLTAGE_TOLERANCE = Millivolts.of(500);
  private static final Current CURRENT_TOLERANCE = Milliamps.of(500);

  private final GenericFlywheelConstants constants;
  private final MotorControllerGroup motors;

  private final String aKitTopic;

  @Getter private GenericFlywheelState currentState;

  @Getter private Setpoint<AngularVelocityUnit> velocityGoal;
  @Getter private Setpoint<VoltageUnit> voltageGoal;
  @Getter private Current currentGoal;

  private final CustomSysIdRoutine<VoltageUnit> voltageCharacterizationRoutine;
  private final CustomSysIdRoutine<CurrentUnit> torqueCharacterizationRoutine;

  /**
   * Creates a flywheel.
   *
   * @param motorFactory how to build the flywheel's motors
   * @param subsystem the subsystem that owns the flywheel, for its log prefix
   * @param constants the flywheel's constants
   * @param name the flywheel's name, appended to the log prefix
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
    this.constants = constants;
    motors = new MotorControllerGroup(motorConfig(constants), motorFactory);

    aKitTopic = subsystem.getName() + "/" + "Flywheel" + name;

    currentState = GenericFlywheelState.IDLE;

    this.velocityGoal = velocityGoal;
    this.voltageGoal = voltageGoal;
    currentGoal = Amps.of(0.0);

    voltageCharacterizationRoutine =
        new CustomSysIdRoutine<>(
            new CustomSysIdRoutine.Config<VoltageUnit>(
                CustomUnits.voltsPerSecond.ofNative(0.5),
                Volts.of(8),
                Seconds.of(24),
                (state) ->
                    Logger.recordOutput(aKitTopic + "/Voltage SysID State", state.toString()),
                Volts),
            new CustomSysIdRoutine.Mechanism<>(
                (volts) -> motors.setControl(voltageRequest(Volts.of(volts.in(Volts)))),
                subsystem));

    torqueCharacterizationRoutine =
        new CustomSysIdRoutine<>(
            new CustomSysIdRoutine.Config<CurrentUnit>(
                CustomUnits.ampsPerSecond.ofNative(5.0),
                Amps.of(40),
                Seconds.of(10),
                (state) ->
                    Logger.recordOutput(
                        aKitTopic + "/Torque Current SysID State", state.toString()),
                Amp),
            new CustomSysIdRoutine.Mechanism<>(
                (amps) ->
                    motors.setControl(new OpenLoopRequest.CurrentOutput(Amps.of(amps.in(Amps)))),
                subsystem));

    constants.motor.slots.forEach(
        (slot, slotConfig) -> slotConfig.gains().onChange(gains -> updateGains(gains, slot)));
    constants.constraints.onChange(this::updateConstraints);
  }

  /**
   * Creates a flywheel with zero velocity and voltage goals.
   *
   * @param motorFactory how to build the flywheel's motors
   * @param subsystem the subsystem that owns the flywheel, for its log prefix
   * @param constants the flywheel's constants
   * @param name the flywheel's name, appended to the log prefix
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
   * @param name the flywheel's name, appended to the log prefix
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

  private static MotorConfig motorConfig(GenericFlywheelConstants constants) {
    return constants.motor.toBuilder()
        .withMotionProfile(motionProfile(constants.constraints))
        .build();
  }

  private static MotionProfileConfig motionProfile(AngularVelocityConstraints constraints) {
    return MotionProfileConfig.trapezoidal(
        constraints.maxVelocity().get(), constraints.maxAcceleration().get());
  }

  private OpenLoopRequest.VoltageOutput voltageRequest(Voltage voltage) {
    return new OpenLoopRequest.VoltageOutput(
        voltage, constants.motor.enableFOC, RequestOptions.DEFAULT);
  }

  @Trace
  public void periodic() {
    motors.periodic(aKitTopic);

    Logger.recordOutput(aKitTopic + "/State", currentState.name());
    Logger.recordOutputMeasure(aKitTopic + "/Velocity Goal", velocityGoal.getSetpoint());
    Logger.recordOutputMeasure(aKitTopic + "/Voltage Goal", voltageGoal.getSetpoint());
    Logger.recordOutputMeasure(aKitTopic + "/Voltage Offset", voltageGoal.getOffset());
    Logger.recordOutputMeasure(aKitTopic + "/Velocity Offset", velocityGoal.getOffset());
    Logger.recordOutput(aKitTopic + "/Current Goal", currentGoal);
    Logger.recordOutput(aKitTopic + "/At Velocity Goal", atVelocityGoal());
    Logger.recordOutput(aKitTopic + "/At Voltage Goal", atVoltageGoal());
    Logger.recordOutput(aKitTopic + "/At Current Goal", atCurrentGoal());

    switch (currentState) {
      case VELOCITY_VOLTAGE_CONTROL ->
          motors.setControl(
              new VelocityRequest.VelocityVoltage(
                  (AngularVelocity) velocityGoal.getNewSetpoint(),
                  RadiansPerSecondPerSecond.of(0.0),
                  Volts.of(0.0),
                  constants.motor.enableFOC,
                  GainSlot.ZERO,
                  RequestOptions.DEFAULT));
      case VELOCITY_TORQUE_CONTROL ->
          motors.setControl(
              new VelocityRequest.VelocityCurrent(
                  (AngularVelocity) velocityGoal.getNewSetpoint(),
                  RadiansPerSecondPerSecond.of(0.0),
                  currentGoal,
                  GainSlot.ONE,
                  RequestOptions.DEFAULT));
      case VOLTAGE_CONTROL ->
          motors.setControl(voltageRequest((Voltage) voltageGoal.getNewSetpoint()));
      case STOP -> motors.setControl(new NeutralRequest.Neutral());
      case IDLE -> {}
    }
  }

  public AngularVelocity getFlywheelVelocity() {
    return motors.leader().velocity;
  }

  public Angle getFlywheelPosition() {
    return motors.leader().position;
  }

  /**
   * Returns each motor's torque current in amps, leader first.
   *
   * @return the torque currents
   */
  public double[] getTorqueCurrent() {
    return motors.telemetry().stream()
        .mapToDouble((MotorTelemetry telemetry) -> telemetry.torqueCurrent.in(Amps))
        .toArray();
  }

  public void setVoltageGoal(Voltage voltageGoal) {
    currentState = GenericFlywheelState.VOLTAGE_CONTROL;
    this.voltageGoal.setSetpoint(voltageGoal);
  }

  public void setVoltageGoal(Setpoint<VoltageUnit> voltageGoal) {
    currentState = GenericFlywheelState.VOLTAGE_CONTROL;
    this.voltageGoal = voltageGoal;
  }

  public void setVelocityGoal(AngularVelocity velocityGoal) {
    currentState = GenericFlywheelState.VELOCITY_VOLTAGE_CONTROL;
    this.velocityGoal.setSetpoint(velocityGoal);
  }

  public void setVelocityGoal(Setpoint<AngularVelocityUnit> velocityGoal) {
    currentState = GenericFlywheelState.VELOCITY_VOLTAGE_CONTROL;
    this.velocityGoal = velocityGoal;
  }

  public void setVelocityGoal(Supplier<AngularVelocity> velocityGoal) {
    currentState = GenericFlywheelState.VELOCITY_VOLTAGE_CONTROL;
    this.velocityGoal.setSetpoint(velocityGoal.get());
  }

  public void setVelocityGoal(AngularVelocity velocityGoal, Current currentGoal) {
    currentState = GenericFlywheelState.VELOCITY_TORQUE_CONTROL;
    this.velocityGoal.setSetpoint(velocityGoal);
    this.currentGoal = currentGoal;
  }

  public void setVelocityGoal(Setpoint<AngularVelocityUnit> velocityGoal, Current currentGoal) {
    currentState = GenericFlywheelState.VELOCITY_TORQUE_CONTROL;
    this.velocityGoal = velocityGoal;
    this.currentGoal = currentGoal;
  }

  public void setVelocityGoal(
      Supplier<AngularVelocity> velocityGoal, Supplier<Current> currentGoal) {
    currentState = GenericFlywheelState.VELOCITY_TORQUE_CONTROL;
    this.velocityGoal.setSetpoint(velocityGoal.get());
    this.currentGoal = currentGoal.get();
  }

  public void stop() {
    currentState = GenericFlywheelState.STOP;
  }

  public boolean atVoltageGoal(Voltage voltageReference) {
    return motors.leader().appliedVoltage.isNear(voltageReference, VOLTAGE_TOLERANCE);
  }

  public boolean atVelocityGoal(AngularVelocity velocityReference) {
    return motors
        .leader()
        .velocity
        .isNear(velocityReference, constants.constraints.goalTolerance().get());
  }

  public boolean atCurrentGoal(Current currentReference) {
    return motors.leader().torqueCurrent.isNear(currentReference, CURRENT_TOLERANCE);
  }

  public boolean atVoltageGoal() {
    return atVoltageGoal((Voltage) voltageGoal.getNewSetpoint());
  }

  public boolean atVelocityGoal() {
    return atVelocityGoal((AngularVelocity) velocityGoal.getNewSetpoint());
  }

  public boolean atCurrentGoal() {
    return atCurrentGoal(currentGoal);
  }

  public Command waitUntilAtGoal() {
    return Commands.waitUntil(this::atVelocityGoal);
  }

  public void updateGains(Gains gains, GainSlot gainSlot) {
    motors.updateGains(gains, gainSlot);
  }

  public void updateConstraints(AngularVelocityConstraints constraints) {
    motors.updateMotionProfile(motionProfile(constraints));
  }

  public Command sysIdRoutineVoltage() {
    return Commands.sequence(
        Commands.runOnce(() -> currentState = GenericFlywheelState.IDLE),
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
        Commands.runOnce(() -> currentState = GenericFlywheelState.IDLE),
        torqueCharacterizationRoutine.dynamic(CustomSysIdRoutine.Direction.kForward),
        Commands.waitSeconds(20.0),
        torqueCharacterizationRoutine.dynamic(CustomSysIdRoutine.Direction.kReverse),
        Commands.waitSeconds(20.0),
        torqueCharacterizationRoutine.quasistatic(CustomSysIdRoutine.Direction.kForward),
        Commands.waitSeconds(20.0),
        torqueCharacterizationRoutine.quasistatic(CustomSysIdRoutine.Direction.kReverse));
  }
}
