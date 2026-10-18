package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.phoenix;

import edu.wpi.team190.gompeilib.io.encoder.phoenix.AbsoluteEncoderIOCANcoder;
import edu.wpi.team190.gompeilib.io.motor.phoenix.MotorIOTalonFX;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixOdometryThread;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveModuleConfig;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveModuleHardware;

/** Builds a swerve module from TalonFX drive and turn motors and a CANcoder. */
public final class SwerveModuleHardwareTalonFX {
  private SwerveModuleHardwareTalonFX() {}

  /**
   * Builds a module's hardware, with both motors' positions sampled by the odometry thread.
   *
   * @param config the module's config
   * @param odometryThread the thread that samples motor positions between cycles
   * @return the hardware
   */
  public static SwerveModuleHardware create(
      SwerveModuleConfig config, PhoenixOdometryThread odometryThread) {
    return new SwerveModuleHardware(
        MotorIOTalonFX.factory(odometryThread),
        MotorIOTalonFX.factory(odometryThread),
        new AbsoluteEncoderIOCANcoder(config.encoder()));
  }
}
