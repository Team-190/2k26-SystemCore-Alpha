package edu.wpi.team190.gompeilib.io.encoder.phoenix;

import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.team190.gompeilib.io.encoder.config.AbsoluteEncoderConfig;
import edu.wpi.team190.gompeilib.io.encoder.config.SensorDirection;
import edu.wpi.team190.gompeilib.io.encoder.inputs.MagnetHealth;

/** Pure conversions between GompeiLib's absolute encoder types and Phoenix6's CANcoder types. */
public final class CANcoderConversions {
  private CANcoderConversions() {}

  /**
   * Builds the device configuration for an encoder.
   *
   * @param config the encoder's config
   * @return the CANcoder configuration
   */
  public static CANcoderConfiguration toConfiguration(AbsoluteEncoderConfig config) {
    CANcoderConfiguration cancoder = new CANcoderConfiguration();
    cancoder.MagnetSensor.SensorDirection =
        config.direction == SensorDirection.CLOCKWISE_POSITIVE
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    cancoder.MagnetSensor.MagnetOffset = config.magnetOffset.in(Rotations);
    cancoder.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        config.discontinuityPoint.in(Rotations);
    return cancoder;
  }

  /**
   * Converts a reported magnet health.
   *
   * @param health the Phoenix6 magnet health
   * @return the magnet health
   */
  public static MagnetHealth toMagnetHealth(MagnetHealthValue health) {
    return switch (health) {
      case Magnet_Green -> MagnetHealth.GREEN;
      case Magnet_Orange -> MagnetHealth.ORANGE;
      case Magnet_Red -> MagnetHealth.RED;
      case Magnet_Invalid -> MagnetHealth.INVALID;
    };
  }
}
