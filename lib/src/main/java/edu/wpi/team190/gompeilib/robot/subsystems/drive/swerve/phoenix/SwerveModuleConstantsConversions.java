package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.phoenix;

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
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveModuleConfig;
import java.util.Map;
import org.wpilib.math.geometry.Translation2d;
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
   * @return the module config
   */
  public static SwerveModuleConfig toModuleConfig(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants,
      CANBusId bus,
      Frequency odometryFrequency,
      Time closedLoopRampPeriod) {
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
            .withSlot(GainSlot.ZERO, new SlotConfig(toGains(constants.DriveMotorGains)))
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
            .withSlot(GainSlot.ZERO, new SlotConfig(toGains(constants.SteerMotorGains)))
            .withMotionProfile(
                new MotionProfileConfig(
                    RotationsPerSecond.of(cruiseRotationsPerSecond),
                    RotationsPerSecondPerSecond.of(cruiseRotationsPerSecond / 0.100),
                    RotationsPerSecondPerSecond.per(Second).of(0.0),
                    Volts.per(RotationsPerSecond).ofNative(0.12 * steerRatio),
                    Volts.per(RotationsPerSecondPerSecond).ofNative(0.1)))
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
        .withDriveInertia(KilogramSquareMeters.of(constants.DriveInertia))
        .withTurnInertia(KilogramSquareMeters.of(constants.SteerInertia))
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
   * Converts slot gains, without publishing them as tunables.
   *
   * @param slot the Phoenix6 slot
   * @return the gains
   */
  public static Gains toGains(Slot0Configs slot) {
    return Gains.builder()
        .withKP(slot.kP)
        .withKI(slot.kI)
        .withKD(slot.kD)
        .withKS(slot.kS)
        .withKV(slot.kV)
        .withKA(slot.kA)
        .withKG(slot.kG)
        .build();
  }

  private static MotorDirection direction(boolean inverted) {
    return inverted ? MotorDirection.CLOCKWISE_POSITIVE : MotorDirection.COUNTER_CLOCKWISE_POSITIVE;
  }
}
