package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.phoenix;

import static edu.wpi.team190.gompeilib.math.CustomUnits.*;
import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import edu.wpi.team190.gompeilib.io.encoder.config.AbsoluteEncoderConfig;
import edu.wpi.team190.gompeilib.io.encoder.config.SensorDirection;
import edu.wpi.team190.gompeilib.io.motor.config.ClosedLoopRamps;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.FeedbackSensor;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorDirection;
import edu.wpi.team190.gompeilib.io.motor.config.NeutralMode;
import edu.wpi.team190.gompeilib.io.motor.config.RemoteSensorMode;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.io.motor.config.TorqueCurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.physics.PlantCharacterization;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveModuleConfig;
import java.util.Map;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.MultUnit;
import org.wpilib.units.PerUnit;
import org.wpilib.units.Unit;
import org.wpilib.units.measure.Frequency;
import org.wpilib.units.measure.Time;

/**
 * Converts the module constants Phoenix Tuner X generates into GompeiLib's vendor-neutral {@link
 * SwerveModuleConfig}.
 */
public final class SwerveModuleConstantsConversions {
  /** The turn motor's torque-current peak and stator limit. */
  public static final double TURN_CURRENT_LIMIT_AMPS = 40.0;

  private SwerveModuleConstantsConversions() {}

  /**
   * Converts one module's constants.
   *
   * @param constants the generated module constants
   * @param bus the bus the module's devices are on
   * @param odometryFrequency how often the odometry thread samples positions
   * @param closedLoopRampPeriod the torque-current closed-loop ramp period
   * @param driveModel the drive motor, to derive the drive plant from the drive inertia
   * @param turnModel the turn motor, to derive the turn plant from the steer inertia
   * @return the module config
   */
  public static SwerveModuleConfig toModuleConfig(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants,
      CANBusId bus,
      Frequency odometryFrequency,
      Time closedLoopRampPeriod,
      DCMotor driveModel,
      DCMotor turnModel) {
    ClosedLoopRamps ramps =
        new ClosedLoopRamps(Seconds.of(0.0), Seconds.of(0.0), closedLoopRampPeriod);
    SignalRates odometryRates =
        new SignalRates(
            SignalRates.DEFAULT.control(),
            SignalRates.DEFAULT.diagnostic(),
            Map.of("position", odometryFrequency));

    MotorConfig drive =
        MotorConfig.builder()
            .withCanId(constants.DriveMotorId)
            .withBus(bus)
            .withDirection(direction(constants.DriveMotorInverted))
            .withNeutralMode(NeutralMode.BRAKE)
            .withCurrentLimits(new CurrentLimits(null, Amps.of(constants.SlipCurrent)))
            .withTorqueCurrentLimits(TorqueCurrentLimits.symmetric(Amps.of(constants.SlipCurrent)))
            .withGearRatio(constants.DriveMotorGearRatio)
            .withSlot(
                GainSlot.ZERO,
                SlotConfig.velocity(
                    toVelocityGains(
                        null, constants.DriveMotorGains, constants.DriveMotorClosedLoopOutput)))
            .withClosedLoopRamps(ramps)
            .withSignalRates(odometryRates)
            .build();

    double steerRatio = constants.SteerMotorGearRatio;
    double cruiseRotationsPerSecond = 100.0 / steerRatio;
    MotorConfig turn =
        MotorConfig.builder()
            .withCanId(constants.SteerMotorId)
            .withBus(bus)
            .withDirection(direction(constants.SteerMotorInverted))
            .withNeutralMode(NeutralMode.BRAKE)
            .withCurrentLimits(new CurrentLimits(null, Amps.of(TURN_CURRENT_LIMIT_AMPS)))
            .withTorqueCurrentLimits(
                TorqueCurrentLimits.symmetric(Amps.of(TURN_CURRENT_LIMIT_AMPS)))
            .withGearRatio(steerRatio)
            .withFeedbackSensor(
                new FeedbackSensor.RemoteEncoder(
                    constants.EncoderId, toRemoteSensorMode(constants.FeedbackSource)))
            .withContinuousWrap(true)
            .withSlot(
                GainSlot.ZERO,
                SlotConfig.position(
                    toPositionGains(
                        null, constants.SteerMotorGains, constants.SteerMotorClosedLoopOutput)))
            .withMotionProfile(
                new MotionProfileConfig(
                    RotationsPerSecond.of(cruiseRotationsPerSecond),
                    RotationsPerSecondPerSecond.of(cruiseRotationsPerSecond / 0.100),
                    RotationsPerSecondCubed.of(0.0),
                    VoltsPerRotationPerSecond.ofNative(0.12 * steerRatio),
                    VoltsPerRotationPerSecondSquared.ofNative(0.1)))
            .withClosedLoopRamps(ramps)
            .withSignalRates(odometryRates)
            .build();

    AbsoluteEncoderConfig.AbsoluteEncoderConfigBuilder encoder =
        AbsoluteEncoderConfig.builder()
            .withCanId(constants.EncoderId)
            .withBus(bus)
            .withDirection(
                constants.EncoderInverted
                    ? SensorDirection.CLOCKWISE_POSITIVE
                    : SensorDirection.COUNTER_CLOCKWISE_POSITIVE)
            .withMagnetOffset(Rotations.of(constants.EncoderOffset));
    if (constants.EncoderInitialConfigs != null) {
      encoder.withDiscontinuityPoint(
          Rotations.of(
              constants.EncoderInitialConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint));
    }

    return SwerveModuleConfig.builder()
        .withDriveMotor(drive)
        .withTurnMotor(turn)
        .withEncoder(encoder.build())
        .withLocation(new Translation2d(constants.LocationX, constants.LocationY))
        // Tuner X gives friction in volts; each plant is in the effort its closed loop outputs
        .withDrivePlant(
            PlantCharacterization.ofInertia(
                    driveModel,
                    constants.DriveMotorGearRatio,
                    KilogramSquareMeters.of(constants.DriveInertia))
                .withKS(Volts.of(constants.DriveFrictionVoltage))
                .build()
                .in(
                    toOutputType(constants.DriveMotorClosedLoopOutput),
                    driveModel,
                    constants.DriveMotorGearRatio,
                    1))
        .withTurnPlant(
            PlantCharacterization.ofInertia(
                    turnModel, steerRatio, KilogramSquareMeters.of(constants.SteerInertia))
                .withKS(Volts.of(constants.SteerFrictionVoltage))
                .build()
                .in(toOutputType(constants.SteerMotorClosedLoopOutput), turnModel, steerRatio, 1))
        .build();
  }

  /**
   * Converts a closed-loop output type.
   *
   * @param type the Phoenix6 output type
   * @return the GompeiLib output type
   */
  public static OutputType toOutputType(ClosedLoopOutputType type) {
    return switch (type) {
      case Voltage -> OutputType.VOLTAGE;
      case TorqueCurrentFOC -> OutputType.CURRENT;
    };
  }

  /**
   * Converts how the steer motor uses its encoder.
   *
   * @param type the Phoenix6 feedback type
   * @return the GompeiLib remote-sensor mode
   * @throws IllegalArgumentException for a feedback type without a CANcoder
   */
  public static RemoteSensorMode toRemoteSensorMode(SteerFeedbackType type) {
    return switch (type) {
      case FusedCANcoder -> RemoteSensorMode.FUSED;
      case SyncCANcoder -> RemoteSensorMode.SYNC;
      case RemoteCANcoder -> RemoteSensorMode.REMOTE;
      default -> throw new IllegalArgumentException("Unsupported steer feedback type: " + type);
    };
  }

  /**
   * Converts a velocity slot, whose gains are per wheel rotation per second in volts or amps.
   *
   * @param prefix where to publish the gains, or null to leave them unpublished
   * @param slot the Phoenix6 slot
   * @param output the output the slot's requests use
   * @return the gains
   */
  public static Gains<?, AngularVelocityUnit, AngularVelocityUnit> toVelocityGains(
      String prefix, Slot0Configs slot, ClosedLoopOutputType output) {
    return switch (output) {
      case Voltage -> velocityGains(Volts, prefix, slot);
      case TorqueCurrentFOC -> velocityGains(Amps, prefix, slot);
    };
  }

  /**
   * Converts a position slot, whose gains are per module rotation in volts or amps.
   *
   * @param prefix where to publish the gains, or null to leave them unpublished
   * @param slot the Phoenix6 slot
   * @param output the output the slot's requests use
   * @return the gains
   */
  public static Gains<?, AngleUnit, AngularVelocityUnit> toPositionGains(
      String prefix, Slot0Configs slot, ClosedLoopOutputType output) {
    return switch (output) {
      case Voltage -> positionGains(Volts, prefix, slot);
      case TorqueCurrentFOC -> positionGains(Amps, prefix, slot);
    };
  }

  // A Phoenix slot holds the output's base unit per rotation (or rotation per second).
  private static <O extends Unit> Gains<O, AngularVelocityUnit, AngularVelocityUnit> velocityGains(
      O output, String prefix, Slot0Configs slot) {
    Gains.Builder<O, AngularVelocityUnit, AngularVelocityUnit> builder =
        Gains.angularVelocity(output)
            .withPrefix(prefix)
            .withKP(PerUnit.combine(output, RotationsPerSecond).ofNative(slot.kP))
            .withKI(
                PerUnit.combine(output, MultUnit.combine(RotationsPerSecond, Seconds))
                    .ofNative(slot.kI))
            .withKD(PerUnit.combine(output, RotationsPerSecondPerSecond).ofNative(slot.kD));
    return feedforward(builder, output, slot).build();
  }

  private static <O extends Unit> Gains<O, AngleUnit, AngularVelocityUnit> positionGains(
      O output, String prefix, Slot0Configs slot) {
    Gains.Builder<O, AngleUnit, AngularVelocityUnit> builder =
        Gains.angularPosition(output)
            .withPrefix(prefix)
            .withKP(PerUnit.combine(output, Rotations).ofNative(slot.kP))
            .withKI(PerUnit.combine(output, MultUnit.combine(Rotations, Seconds)).ofNative(slot.kI))
            .withKD(PerUnit.combine(output, RotationsPerSecond).ofNative(slot.kD));
    return feedforward(builder, output, slot).build();
  }

  // A unit's of() returns that unit's own measure type, which is a Measure<O> for an O.
  @SuppressWarnings("unchecked")
  private static <O extends Unit, Q extends Unit>
      Gains.Builder<O, Q, AngularVelocityUnit> feedforward(
          Gains.Builder<O, Q, AngularVelocityUnit> builder, O output, Slot0Configs slot) {
    return builder
        .withKS((Measure<O>) output.of(slot.kS))
        .withKV(PerUnit.combine(output, RotationsPerSecond).ofNative(slot.kV))
        .withKA(PerUnit.combine(output, RotationsPerSecondPerSecond).ofNative(slot.kA))
        .withKG((Measure<O>) output.of(slot.kG));
  }

  private static MotorDirection direction(boolean inverted) {
    return inverted ? MotorDirection.CLOCKWISE_POSITIVE : MotorDirection.COUNTER_CLOCKWISE_POSITIVE;
  }
}
