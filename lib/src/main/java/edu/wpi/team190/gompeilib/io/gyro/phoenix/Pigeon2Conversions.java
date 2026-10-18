package edu.wpi.team190.gompeilib.io.gyro.phoenix;

import static org.wpilib.units.Units.Degrees;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.team190.gompeilib.io.gyro.config.GyroConfig;

/** Pure conversions between GompeiLib's gyro types and Phoenix6's Pigeon2 types. */
public final class Pigeon2Conversions {
  private Pigeon2Conversions() {}

  /**
   * Builds the device configuration for a gyro.
   *
   * @param config the gyro's config
   * @return the Pigeon2 configuration
   */
  public static Pigeon2Configuration toConfiguration(GyroConfig config) {
    Pigeon2Configuration pigeon = new Pigeon2Configuration();
    pigeon.MountPose.MountPoseYaw = config.mountPoseYaw.in(Degrees);
    pigeon.MountPose.MountPosePitch = config.mountPosePitch.in(Degrees);
    pigeon.MountPose.MountPoseRoll = config.mountPoseRoll.in(Degrees);
    pigeon.Pigeon2Features.EnableCompass = config.enableCompass;
    pigeon.Pigeon2Features.DisableTemperatureCompensation = config.disableTemperatureCompensation;
    pigeon.Pigeon2Features.DisableNoMotionCalibration = config.disableNoMotionCalibration;
    return pigeon;
  }
}
