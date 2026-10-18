package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.io.encoder.inputs.AbsoluteEncoderTelemetryAutoLogged;
import edu.wpi.team190.gompeilib.io.encoder.interfaces.AbsoluteEncoderIO;
import edu.wpi.team190.gompeilib.io.motor.MotorControllerGroup;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.motor.outputs.PositionRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.motor.outputs.VelocityRequest;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDriveConstants.DriveConfig;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.util.Alert;

/** One swerve module: a drive motor, a turn motor, and an absolute encoder on the turn axis. */
public class SwerveModule {
  // Alert ids are unique per process, so a module rebuilt with the same index (e.g. in tests)
  // reuses its existing alerts instead of allocating duplicates.
  private static final Map<String, Alert> alerts = new ConcurrentHashMap<>();

  private final SwerveDriveConstants driveConstants;
  private final SwerveModuleConfig config;
  private final MotorControllerGroup drive;
  private final MotorControllerGroup turn;
  private final AbsoluteEncoderIO encoder;
  private final AbsoluteEncoderTelemetryAutoLogged encoderTelemetry =
      new AbsoluteEncoderTelemetryAutoLogged();
  private final String aKitTopic;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  @Getter private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
  @Getter private double[] odometryTimestamps = new double[] {};

  /**
   * Creates a module.
   *
   * @param driveConstants the drive's constants
   * @param config the module's config
   * @param hardware the module's components
   * @param index the module's index, for its log prefix and alerts
   */
  public SwerveModule(
      SwerveDriveConstants driveConstants,
      SwerveModuleConfig config,
      SwerveModuleHardware hardware,
      int index) {
    this.driveConstants = driveConstants;
    this.config = config;
    DriveConfig driveConfig = driveConstants.driveConfig;
    requireSlot(
        config.driveMotor(),
        SlotConfig.Velocity.class,
        driveConfig.driveClosedLoopOutputType(),
        "Module " + index + " drive");
    requireSlot(
        config.turnMotor(),
        SlotConfig.Position.class,
        driveConfig.steerClosedLoopOutputType(),
        "Module " + index + " turn");
    drive = new MotorControllerGroup(config.driveMotor(), hardware.driveMotor());
    turn = new MotorControllerGroup(config.turnMotor(), hardware.turnMotor());
    encoder = hardware.encoder();
    aKitTopic = "Drive/Module" + index;

    driveDisconnectedAlert =
        getAlert(
            "SwerveModule" + index + "DriveDisconnected",
            "Disconnected drive motor on module " + index + ".");
    turnDisconnectedAlert =
        getAlert(
            "SwerveModule" + index + "TurnDisconnected",
            "Disconnected turn motor on module " + index + ".");
    turnEncoderDisconnectedAlert =
        getAlert(
            "SwerveModule" + index + "TurnEncoderDisconnected",
            "Disconnected turn encoder on module " + index + ".");
  }

  /**
   * Checks a motor's slot {@code ZERO}, if set, against the loop and output type the module runs it
   * with.
   *
   * @param motor the motor
   * @param kind the slot kind the module's requests use
   * @param outputType the output type the module's requests use
   * @param name the motor, for the error message
   * @throws IllegalArgumentException if the slot doesn't match
   */
  static void requireSlot(
      MotorConfig motor, Class<? extends SlotConfig> kind, OutputType outputType, String name) {
    SlotConfig slot = motor.slots.get(GainSlot.ZERO);
    if (slot == null) {
      return;
    }
    if (!kind.isInstance(slot)) {
      throw new IllegalArgumentException(
          name + " slot ZERO must be a " + kind.getSimpleName() + " slot, was " + slot);
    }
    SlotConfig.requireOutputType(slot, outputType, name);
  }

  private static Alert getAlert(String id, String text) {
    return alerts.computeIfAbsent(id, key -> new Alert(key, text, Alert.Level.HIGH));
  }

  private double wheelRadiusMeters() {
    return driveConstants.driveConfig.wheelRadiusMeters();
  }

  @Trace
  public void updateInputs() {
    drive.periodic(aKitTopic + "/Drive");
    turn.periodic(aKitTopic + "/Turn");
    encoder.updateInputs(encoderTelemetry);
    Logger.processInputs(aKitTopic + "/Encoder", encoderTelemetry);
  }

  @Trace
  public void periodic() {
    // Calculate positions for odometry. Both motors are sampled together.
    MotorTelemetry driveTelemetry = drive.leader();
    MotorTelemetry turnTelemetry = turn.leader();
    int sampleCount =
        Math.min(
            driveTelemetry.odometryTimestamps.length,
            Math.min(
                driveTelemetry.odometryPositions.length, turnTelemetry.odometryPositions.length));
    odometryTimestamps = new double[sampleCount];
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      odometryTimestamps[i] = driveTelemetry.odometryTimestamps[i];
      double positionMeters = driveTelemetry.odometryPositions[i].in(Radians) * wheelRadiusMeters();
      odometryPositions[i] =
          new SwerveModulePosition(
              positionMeters, new Rotation2d(turnTelemetry.odometryPositions[i]));
    }

    // Update alerts
    driveDisconnectedAlert.set(!drive.isConnected());
    turnDisconnectedAlert.set(!turn.isConnected());
    turnEncoderDisconnectedAlert.set(!encoderTelemetry.connected);
  }

  /**
   * Runs the module with the specified setpoint state.
   *
   * @return The optimized state that was actually applied, for callers that want to log it.
   */
  @Trace
  public SwerveModuleVelocity runSetpoint(
      SwerveModuleVelocity state, SwerveModuleVelocity torqueFeedforward) {
    // Optimize velocity setpoint. SwerveModuleVelocity is immutable now, so these return new
    // instances instead of mutating in place.
    state = state.optimize(getAngle());
    state = state.cosineScale(getAngle());

    DCMotor driveModel = driveConstants.driveConfig.driveModel();
    double motorTorqueNewtonMeters = torqueFeedforward.velocity / config.driveMotor().gearRatio;
    double feedforwardAmps = driveModel.getCurrent(motorTorqueNewtonMeters);
    var velocity = RadiansPerSecond.of(state.velocity / wheelRadiusMeters());
    if (driveConstants.driveConfig.driveClosedLoopOutputType() == OutputType.CURRENT) {
      drive.setControl(
          new VelocityRequest.VelocityCurrent(
              velocity,
              RadiansPerSecondPerSecond.of(0.0),
              Amps.of(feedforwardAmps),
              GainSlot.ZERO,
              RequestOptions.DEFAULT));
    } else {
      drive.setControl(
          new VelocityRequest.VelocityVoltage(
              velocity,
              RadiansPerSecondPerSecond.of(0.0),
              Volts.of(feedforwardAmps * driveModel.R),
              config.driveMotor().enableFOC,
              GainSlot.ZERO,
              RequestOptions.DEFAULT));
    }
    setTurnPosition(state.angle);
    return state;
  }

  private void setTurnPosition(Rotation2d angle) {
    Angle position = angle.getMeasure();
    if (driveConstants.driveConfig.steerClosedLoopOutputType() == OutputType.CURRENT) {
      turn.setControl(
          new PositionRequest.ProfiledPositionCurrent(
              position, Amps.of(0.0), GainSlot.ZERO, RequestOptions.DEFAULT));
    } else {
      turn.setControl(
          new PositionRequest.ProfiledPositionVoltage(
              position,
              Volts.of(0.0),
              config.turnMotor().enableFOC,
              GainSlot.ZERO,
              RequestOptions.DEFAULT));
    }
  }

  /** Runs the drive motor with an open-loop request while controlling to zero degrees. */
  @Trace
  public void runCharacterization(OpenLoopRequest request) {
    drive.setControl(request);
    setTurnPosition(new Rotation2d());
  }

  /** Disables all outputs to motors. */
  @Trace
  public void stop() {
    drive.setControl(new OpenLoopRequest.CurrentOutput(Amps.of(0.0)));
    turn.setControl(new OpenLoopRequest.CurrentOutput(Amps.of(0.0)));
  }

  /** Returns the current turn angle of the module. */
  @Trace
  public Rotation2d getAngle() {
    return new Rotation2d(turn.leader().position);
  }

  /** Returns the current drive position of the module in meters. */
  @Trace
  public double getPositionMeters() {
    return drive.leader().position.in(Radians) * wheelRadiusMeters();
  }

  /** Returns the current drive velocity of the module in meters per second. */
  @Trace
  public double getVelocityMetersPerSec() {
    return drive.leader().velocity.in(RadiansPerSecond) * wheelRadiusMeters();
  }

  /** Returns the module position (turn angle and drive position). */
  @Trace
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  @Trace
  public SwerveModuleVelocity getState() {
    return new SwerveModuleVelocity(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module position in radians. */
  @Trace
  public double getWheelRadiusCharacterizationPosition() {
    return drive.leader().position.in(Radians);
  }

  /** Returns the module velocity in rotations/sec. */
  @Trace
  public double getFFCharacterizationVelocity() {
    return drive.leader().velocity.in(RotationsPerSecond);
  }

  /**
   * Updates the drive motor's closed-loop gains for a slot.
   *
   * @param gains the gains to apply
   * @param slot the target gain slot
   */
  @Trace
  public void updateDriveGains(
      Gains<?, AngularVelocityUnit, AngularVelocityUnit> gains, GainSlot slot) {
    drive.updateGains(SlotConfig.velocity(gains), slot);
  }

  /**
   * Updates the turn motor's closed-loop gains for a slot.
   *
   * @param gains the gains to apply
   * @param slot the target gain slot
   */
  @Trace
  public void updateTurnGains(Gains<?, AngleUnit, AngularVelocityUnit> gains, GainSlot slot) {
    turn.updateGains(SlotConfig.position(gains), slot);
  }

  /**
   * Updates the stator current limits, keeping each motor's supply limit.
   *
   * @param driveCurrentLimit The drive current limit in amps.
   * @param turnCurrentLimit The turn current limit in amps.
   */
  @Trace
  public void updateCurrentLimits(double driveCurrentLimit, double turnCurrentLimit) {
    drive.updateCurrentLimits(
        new CurrentLimits(
            config.driveMotor().currentLimits.supplyCurrentLimit(), Amps.of(driveCurrentLimit)));
    turn.updateCurrentLimits(
        new CurrentLimits(
            config.turnMotor().currentLimits.supplyCurrentLimit(), Amps.of(turnCurrentLimit)));
  }
}
