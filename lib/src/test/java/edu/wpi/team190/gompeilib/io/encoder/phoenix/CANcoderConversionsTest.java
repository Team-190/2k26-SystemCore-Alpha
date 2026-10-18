package edu.wpi.team190.gompeilib.io.encoder.phoenix;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.encoder.config.AbsoluteEncoderConfig;
import edu.wpi.team190.gompeilib.io.encoder.config.SensorDirection;
import edu.wpi.team190.gompeilib.io.encoder.inputs.MagnetHealth;
import org.junit.jupiter.api.Test;

public class CANcoderConversionsTest {

  private static AbsoluteEncoderConfig.AbsoluteEncoderConfigBuilder config() {
    return AbsoluteEncoderConfig.builder().withCanId(1).withBus(CANBusId.named("rio"));
  }

  @Test
  public void testToConfiguration() {
    CANcoderConfiguration cancoder =
        CANcoderConversions.toConfiguration(
            config()
                .withDirection(SensorDirection.CLOCKWISE_POSITIVE)
                .withMagnetOffset(Rotations.of(0.125))
                .withDiscontinuityPoint(Rotations.of(1.0))
                .build());

    assertEquals(SensorDirectionValue.Clockwise_Positive, cancoder.MagnetSensor.SensorDirection);
    assertEquals(0.125, cancoder.MagnetSensor.MagnetOffset, 1e-9);
    assertEquals(1.0, cancoder.MagnetSensor.AbsoluteSensorDiscontinuityPoint, 1e-9);
  }

  @Test
  public void testDefaultConfigurationMatchesTheDevice() {
    CANcoderConfiguration converted = CANcoderConversions.toConfiguration(config().build());
    CANcoderConfiguration factory = new CANcoderConfiguration();

    assertEquals(factory.MagnetSensor.SensorDirection, converted.MagnetSensor.SensorDirection);
    assertEquals(factory.MagnetSensor.MagnetOffset, converted.MagnetSensor.MagnetOffset);
    assertEquals(
        factory.MagnetSensor.AbsoluteSensorDiscontinuityPoint,
        converted.MagnetSensor.AbsoluteSensorDiscontinuityPoint);
  }

  @Test
  public void testToMagnetHealth() {
    assertEquals(
        MagnetHealth.GREEN, CANcoderConversions.toMagnetHealth(MagnetHealthValue.Magnet_Green));
    assertEquals(
        MagnetHealth.ORANGE, CANcoderConversions.toMagnetHealth(MagnetHealthValue.Magnet_Orange));
    assertEquals(
        MagnetHealth.RED, CANcoderConversions.toMagnetHealth(MagnetHealthValue.Magnet_Red));
    assertEquals(
        MagnetHealth.INVALID, CANcoderConversions.toMagnetHealth(MagnetHealthValue.Magnet_Invalid));
  }
}
