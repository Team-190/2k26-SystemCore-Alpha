package edu.wpi.team190.gompeilib.subsystems.extension;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.io.motor.MotorControllerGroup;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.SoftLimits;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.PositionRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.physics.ElevatorMechanismSim;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import edu.wpi.team190.gompeilib.logging.Trace;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Voltage;

/**
 * A linear extension driven through a drum: one leader motor and any number of followers. The
 * motors work in drum rotations; the extension converts to and from travel through the drum radius.
 */
public class Extension {
  private static final Voltage VOLTAGE_TOLERANCE = Millivolts.of(500);

  private final MotorControllerGroup motors;

  private final String aKitTopic;

  private ExtensionState currentState;

  private Setpoint<VoltageUnit> voltageGoal;
  @Getter private Setpoint<DistanceUnit> positionGoal;
  @Getter private GainSlot gainSlot = GainSlot.ZERO;

  private final SysIdRoutine characterizationRoutine;

  @Getter public final ExtensionConstants constants;

  /**
   * Creates an extension.
   *
   * @param constants the extension's constants
   * @param subsystem the subsystem that owns the extension, for its log prefix
   * @param index the extension's index, appended to the log prefix
   * @param motorFactory how to build the extension's motors
   * @param positionGoal the initial position goal
   * @param voltageGoal the initial voltage goal
   */
  public Extension(
      ExtensionConstants constants,
      Subsystem subsystem,
      int index,
      MotorIOFactory motorFactory,
      Setpoint<DistanceUnit> positionGoal,
      Setpoint<VoltageUnit> voltageGoal) {
    this.constants = constants;
    motors = new MotorControllerGroup(motorConfig(constants), motorFactory);

    aKitTopic = subsystem.getName() + "/Extension" + index;

    currentState = ExtensionState.IDLE;

    this.positionGoal = positionGoal;
    this.voltageGoal = voltageGoal;

    characterizationRoutine = createCharacterizationRoutine(subsystem, this);

    constants.motor.slots.forEach(
        (slot, slotConfig) -> slotConfig.gains().onChange(gains -> updateGains(gains, slot)));
    constants.constraints.onChange(this::updateConstraints);
  }

  /**
   * Creates an extension with a position goal at zero, limited to its travel, and a 0 V goal.
   *
   * @param constants the extension's constants
   * @param subsystem the subsystem that owns the extension, for its log prefix
   * @param index the extension's index, appended to the log prefix
   * @param motorFactory how to build the extension's motors
   */
  public Extension(
      ExtensionConstants constants, Subsystem subsystem, int index, MotorIOFactory motorFactory) {
    this(
        constants,
        subsystem,
        index,
        motorFactory,
        new Setpoint<>(
            Meters.of(0),
            constants.heightOffsetStep,
            constants.extensionParameters.minLength(),
            constants.extensionParameters.maxLength()),
        new Setpoint<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12)));
  }

  /**
   * Creates an extension with a 0 V goal.
   *
   * @param constants the extension's constants
   * @param subsystem the subsystem that owns the extension, for its log prefix
   * @param index the extension's index, appended to the log prefix
   * @param motorFactory how to build the extension's motors
   * @param positionGoal the initial position goal
   */
  public Extension(
      ExtensionConstants constants,
      Subsystem subsystem,
      int index,
      MotorIOFactory motorFactory,
      Setpoint<DistanceUnit> positionGoal) {
    this(
        constants,
        subsystem,
        index,
        motorFactory,
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

  private static MotorConfig motorConfig(ExtensionConstants constants) {
    double radius = constants.drumRadius.in(Meters);
    return constants.motor.toBuilder()
        .withSoftLimits(
            SoftLimits.between(
                toAngle(constants.extensionParameters.minLength(), radius),
                toAngle(constants.extensionParameters.maxLength(), radius)))
        .withMotionProfile(motionProfile(constants.constraints, radius))
        .build();
  }

  private static MotionProfileConfig motionProfile(LinearConstraints constraints, double radius) {
    LinearVelocity velocity = constraints.maxVelocity().get();
    LinearAcceleration acceleration = constraints.maxAcceleration().get();
    AngularVelocity cruise = RadiansPerSecond.of(velocity.in(MetersPerSecond) / radius);
    AngularAcceleration angular =
        RadiansPerSecondPerSecond.of(acceleration.in(MetersPerSecondPerSecond) / radius);
    return MotionProfileConfig.trapezoidal(cruise, angular);
  }

  private static Angle toAngle(Distance distance, double radius) {
    return Radians.of(distance.in(Meters) / radius);
  }

  private Angle toAngle(Distance distance) {
    return toAngle(distance, constants.drumRadius.in(Meters));
  }

  private Distance toDistance(Angle angle) {
    return Meters.of(angle.in(Radians) * constants.drumRadius.in(Meters));
  }

  private LinearVelocity toLinearVelocity(AngularVelocity velocity) {
    return MetersPerSecond.of(velocity.in(RadiansPerSecond) * constants.drumRadius.in(Meters));
  }

  private void setVoltage(Voltage voltage) {
    motors.setControl(
        new OpenLoopRequest.VoltageOutput(
            voltage, constants.motor.enableFOC, RequestOptions.DEFAULT));
  }

  @Trace
  public void periodic() {
    motors.periodic(aKitTopic);

    Logger.recordOutput(aKitTopic + "/State", currentState.name());
    Logger.recordOutputMeasure(aKitTopic + "/Voltage Goal", voltageGoal.getSetpoint());
    Logger.recordOutputMeasure(aKitTopic + "/Position Goal", positionGoal.getSetpoint());
    Logger.recordOutputMeasure(aKitTopic + "/Voltage Offset", voltageGoal.getOffset());
    Logger.recordOutputMeasure(aKitTopic + "/Position Offset", positionGoal.getOffset());
    Logger.recordOutputMeasure(aKitTopic + "/Position", getExtensionPosition());
    Logger.recordOutputMeasure(aKitTopic + "/Velocity", getVelocity());
    Logger.recordOutput(aKitTopic + "/At Voltage Goal", atVoltageGoal());
    Logger.recordOutput(aKitTopic + "/At Position Goal", atPositionGoal());

    switch (currentState) {
      case OPEN_LOOP_VOLTAGE_CONTROL -> setVoltage((Voltage) voltageGoal.getNewSetpoint());
      case CLOSED_LOOP_POSITION_CONTROL ->
          motors.setControl(
              new PositionRequest.ProfiledPositionVoltage(
                  toAngle((Distance) positionGoal.getNewSetpoint()),
                  Volts.of(0.0),
                  constants.motor.enableFOC,
                  gainSlot,
                  RequestOptions.DEFAULT));
      case IDLE -> {}
    }
  }

  public Distance getExtensionPosition() {
    return toDistance(motors.leader().position);
  }

  public Current getTorqueCurrent() {
    return motors.leader().torqueCurrent;
  }

  public LinearVelocity getVelocity() {
    return toLinearVelocity(motors.leader().velocity);
  }

  public LinearVelocity getVelocitySetpoint() {
    return toLinearVelocity(motors.leader().closedLoopPositionReferenceSlope);
  }

  public void setVoltageGoal(Voltage voltageGoal) {
    currentState = ExtensionState.OPEN_LOOP_VOLTAGE_CONTROL;
    this.voltageGoal.setSetpoint(voltageGoal);
  }

  public void setPositionGoal(Distance positionGoal) {
    currentState = ExtensionState.CLOSED_LOOP_POSITION_CONTROL;
    this.positionGoal.setSetpoint(positionGoal);
  }

  public void setVoltageGoal(Setpoint<VoltageUnit> voltageGoal) {
    currentState = ExtensionState.OPEN_LOOP_VOLTAGE_CONTROL;
    this.voltageGoal = voltageGoal;
  }

  public void setPositionGoal(Setpoint<DistanceUnit> positionGoal) {
    currentState = ExtensionState.CLOSED_LOOP_POSITION_CONTROL;
    this.positionGoal = positionGoal;
  }

  public boolean atVoltageGoal(Voltage voltageReference) {
    return voltageGoal.getNewSetpoint().isNear(voltageReference, VOLTAGE_TOLERANCE);
  }

  public boolean atPositionGoal(Distance positionReference) {
    return getExtensionPosition()
        .isNear(positionReference, constants.constraints.goalTolerance().get());
  }

  public boolean atVoltageGoal() {
    return atVoltageGoal((Voltage) voltageGoal.getNewSetpoint());
  }

  public boolean atPositionGoal() {
    return atPositionGoal((Distance) positionGoal.getNewSetpoint());
  }

  public void setPosition(Distance position) {
    motors.setPosition(toAngle(position));
  }

  /**
   * Sets the gain slot position control uses from the next cycle on.
   *
   * @param gainSlot the slot
   */
  public void setGainSlot(GainSlot gainSlot) {
    this.gainSlot = gainSlot;
  }

  public Command waitUntilAtGoal() {
    return Commands.waitUntil(this::atPositionGoal);
  }

  public void updateGains(Gains gains, GainSlot slot) {
    motors.updateGains(gains, slot);
  }

  public void updateConstraints(LinearConstraints constraints) {
    motors.updateMotionProfile(motionProfile(constraints, constants.drumRadius.in(Meters)));
  }

  public Command runSysIdRoutine() {
    return runSysIdRoutine(characterizationRoutine, this);
  }

  /** Characterizes several extensions owned by the same subsystem in one pass. */
  public static Command runSysIdRoutine(Subsystem subsystem, Extension... extensions) {
    return runSysIdRoutine(createCharacterizationRoutine(subsystem, extensions), extensions);
  }

  private static SysIdRoutine createCharacterizationRoutine(
      Subsystem subsystem, Extension... extensions) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(1).per(Second),
            Volts.of(3),
            Seconds.of(3),
            (state) -> {
              for (Extension extension : extensions) {
                Logger.recordOutput(extension.aKitTopic + "/SysIdState", state.toString());
              }
            }),
        new SysIdRoutine.Mechanism(
            (voltage) -> {
              for (Extension extension : extensions) {
                extension.setVoltage(voltage);
              }
            },
            null,
            subsystem));
  }

  /**
   * Returns true once any extension has reached the end stop in the direction of travel. The check
   * is one-sided so a fast dynamic test can't overshoot the tolerance band and keep driving.
   */
  private static boolean anyAtEndStop(SysIdRoutine.Direction direction, Extension... extensions) {
    for (Extension extension : extensions) {
      double position = extension.getExtensionPosition().in(Meters);
      double tolerance = extension.constants.constraints.goalTolerance().get().in(Meters);
      ExtensionConstants.ExtensionParameters parameters = extension.constants.extensionParameters;
      if (direction == SysIdRoutine.Direction.FORWARD
          ? position >= parameters.maxLength().in(Meters) - tolerance
          : position <= parameters.minLength().in(Meters) + tolerance) {
        return true;
      }
    }
    return false;
  }

  public Command resetExtensionZero() {
    return Commands.sequence(
        Commands.runOnce(() -> currentState = ExtensionState.IDLE),
        Commands.run(() -> setVoltage(constants.zeroVoltage.times(-1)))
            .until(
                () ->
                    getTorqueCurrent()
                        .isNear(constants.zeroCurrentThreshold, constants.zeroCurrentEpsilon)),
        Commands.runOnce(
            () -> {
              setVoltage(Volts.zero());
              setPosition(constants.extensionParameters.minLength());
            }));
  }

  private static Command runSysIdRoutine(SysIdRoutine routine, Extension... extensions) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              for (Extension extension : extensions) {
                extension.currentState = ExtensionState.IDLE;
              }
            }),
        routine
            .quasistatic(SysIdRoutine.Direction.FORWARD)
            .until(() -> anyAtEndStop(SysIdRoutine.Direction.FORWARD, extensions)),
        Commands.waitSeconds(1.0),
        routine
            .quasistatic(SysIdRoutine.Direction.REVERSE)
            .until(() -> anyAtEndStop(SysIdRoutine.Direction.REVERSE, extensions)),
        Commands.waitSeconds(1.0),
        routine
            .dynamic(SysIdRoutine.Direction.FORWARD)
            .until(() -> anyAtEndStop(SysIdRoutine.Direction.FORWARD, extensions)),
        Commands.waitSeconds(1.0),
        routine
            .dynamic(SysIdRoutine.Direction.REVERSE)
            .until(() -> anyAtEndStop(SysIdRoutine.Direction.REVERSE, extensions)));
  }
}
