package edu.wpi.team190.gompeilib.io.gyro.phoenix;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Degrees;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.gyro.config.GyroConfig;
import org.junit.jupiter.api.Test;

public class Pigeon2ConversionsTest {

  @Test
  public void testToConfiguration() {
    Pigeon2Configuration pigeon =
        Pigeon2Conversions.toConfiguration(
            GyroConfig.builder()
                .withCanId(1)
                .withBus(CANBusId.named("Drive"))
                .withMountPoseYaw(Degrees.of(90.0))
                .withMountPosePitch(Degrees.of(1.0))
                .withMountPoseRoll(Degrees.of(-2.0))
                .withEnableCompass(true)
                .withDisableTemperatureCompensation(true)
                .withDisableNoMotionCalibration(true)
                .build());

    assertEquals(90.0, pigeon.MountPose.MountPoseYaw, 1e-9);
    assertEquals(1.0, pigeon.MountPose.MountPosePitch, 1e-9);
    assertEquals(-2.0, pigeon.MountPose.MountPoseRoll, 1e-9);
    assertTrue(pigeon.Pigeon2Features.EnableCompass);
    assertTrue(pigeon.Pigeon2Features.DisableTemperatureCompensation);
    assertTrue(pigeon.Pigeon2Features.DisableNoMotionCalibration);
  }

  @Test
  public void testDefaultConfigurationMatchesTheDevice() {
    Pigeon2Configuration converted =
        Pigeon2Conversions.toConfiguration(
            GyroConfig.builder().withCanId(1).withBus(CANBusId.named("Drive")).build());
    Pigeon2Configuration factory = new Pigeon2Configuration();

    assertEquals(factory.MountPose.MountPoseYaw, converted.MountPose.MountPoseYaw);
    assertEquals(factory.Pigeon2Features.EnableCompass, converted.Pigeon2Features.EnableCompass);
  }
}
