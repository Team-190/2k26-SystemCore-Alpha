package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve;

import edu.wpi.team190.gompeilib.io.encoder.interfaces.AbsoluteEncoderIO;
import edu.wpi.team190.gompeilib.io.encoder.sim.AbsoluteEncoderIOSim;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.sim.MotorIOSim;
import edu.wpi.team190.gompeilib.io.physics.DCMotorMechanismSim;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import java.util.Objects;
import org.wpilib.math.system.DCMotor;

/**
 * The components one swerve module is built from.
 *
 * @param driveMotor how to build the drive motor
 * @param turnMotor how to build the turn motor
 * @param encoder the turn axis's absolute encoder
 */
public record SwerveModuleHardware(
    MotorIOFactory driveMotor, MotorIOFactory turnMotor, AbsoluteEncoderIO encoder) {
  /** Hardware for log replay: it reads nothing and applies no requests. */
  public static final SwerveModuleHardware REPLAY =
      new SwerveModuleHardware(
          MotorIO.REPLAY_FACTORY, MotorIO.REPLAY_FACTORY, new AbsoluteEncoderIO() {});

  public SwerveModuleHardware {
    Objects.requireNonNull(driveMotor, "driveMotor");
    Objects.requireNonNull(turnMotor, "turnMotor");
    Objects.requireNonNull(encoder, "encoder");
  }

  /**
   * Returns vendor-neutral simulated hardware for a module, using the drive's motor models.
   *
   * @param config the module's config
   * @param driveConfig the drive's config, for its drive and turn motor models
   * @return the hardware
   */
  public static SwerveModuleHardware sim(
      SwerveModuleConfig config, SwerveDriveConstants.DriveConfig driveConfig) {
    return sim(config, driveConfig.driveModel(), driveConfig.turnModel());
  }

  /**
   * Returns vendor-neutral simulated hardware for a module.
   *
   * @param config the module's config
   * @param driveModel the drive motor
   * @param turnModel the turn motor
   * @return the hardware
   */
  public static SwerveModuleHardware sim(
      SwerveModuleConfig config, DCMotor driveModel, DCMotor turnModel) {
    MechanismSim drive =
        new DCMotorMechanismSim(driveModel, config.driveInertia(), config.driveMotor().gearRatio);
    MechanismSim turn =
        new DCMotorMechanismSim(turnModel, config.turnInertia(), config.turnMotor().gearRatio);
    return new SwerveModuleHardware(
        MotorIOSim.factory(drive),
        MotorIOSim.factory(turn),
        new AbsoluteEncoderIOSim(config.encoder(), turn, 1.0));
  }
}
