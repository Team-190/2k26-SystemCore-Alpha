package first.robot.v3_Horse_CV2_TR;

import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIO;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIOPigeon2;
import edu.wpi.team190.gompeilib.core.robot.RobotContainer;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIO;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOSim;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.extension.ExtensionIO;
import edu.wpi.team190.gompeilib.subsystems.extension.ExtensionIOSim;
import edu.wpi.team190.gompeilib.subsystems.extension.ExtensionIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIOSim;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.hood.GenericHoodIO;
import edu.wpi.team190.gompeilib.subsystems.generic.hood.GenericHoodIOSim;
import edu.wpi.team190.gompeilib.subsystems.generic.hood.GenericHoodIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOSim;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.vision.Vision;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraStaticLimelight;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOLimelight;
import first.robot.Constants;
import first.robot.FieldConstants;
import first.robot.RobotConfig;
import first.robot.v3_Horse_CV2_TR.commands.DriveCommands;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_TR_Intake;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_TR_IntakeConstants;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TR_RollerFloor;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TR_RollerFloorConstants;
import first.robot.v3_Horse_CV2_TR.subsystems.shooter.V3_Horse_CV2_TRShooter;
import first.robot.v3_Horse_CV2_TR.subsystems.shooter.V3_Horse_CV2_TRShooterConstants;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;
import org.wpilib.command2.Command;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.math.geometry.Rotation2d;

public class V3_Horse_CV2_TRRobotContainer implements RobotContainer {
  private SwerveDrive drive;
  private V3_Horse_CV2_TR_RollerFloor rollerFloor;
  private V3_Horse_CV2_TR_Intake intake;
  private V3_Horse_CV2_TRShooter shooter;
  private Vision vision;
  private final LoggedNetworkChooser<Command> autoChooser;
  private final CommandNiDsXboxController driver;

  public V3_Horse_CV2_TRRobotContainer() {
    driver = new CommandNiDsXboxController(0);
    if (Constants.getMode() != RobotMode.REPLAY) {
      switch (RobotConfig.ROBOT) {
        case V3_Horse_CV2_TR:
          drive =
              new SwerveDrive(
                  V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS,
                  new GyroIOPigeon2(
                      V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS,
                      V3_Horse_CV2_TRRobotState::setHeadingUpdateTimestamp),
                  new SwerveModuleIOTalonFX(
                      V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS,
                      V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS.driveConfig.frontLeft()),
                  new SwerveModuleIOTalonFX(
                      V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS,
                      V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS.driveConfig.frontRight()),
                  new SwerveModuleIOTalonFX(
                      V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS,
                      V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS.driveConfig.backLeft()),
                  new SwerveModuleIOTalonFX(
                      V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS,
                      V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS.driveConfig.backRight()),
                  V3_Horse_CV2_TRRobotState::getGlobalPose,
                  V3_Horse_CV2_TRRobotState::resetPose);
          rollerFloor =
              new V3_Horse_CV2_TR_RollerFloor(
                  new GenericRollerIOTalonFX(
                      V3_Horse_CV2_TR_RollerFloorConstants.ROLLER_FLOOR_CONSTANTS));
          intake =
              new V3_Horse_CV2_TR_Intake(
                  new GenericRollerIOTalonFX(
                      V3_Horse_CV2_TR_IntakeConstants.LEFT_INTAKE_ROLLER_CONSTANTS),
                  new GenericRollerIOTalonFX(
                      V3_Horse_CV2_TR_IntakeConstants.RIGHT_INTAKE_ROLLER_CONSTANTS),
                  new GenericRollerIOTalonFX(
                      V3_Horse_CV2_TR_IntakeConstants.KICKER_ROLLER_CONSTANTS),
                  new ExtensionIOTalonFX(V3_Horse_CV2_TR_IntakeConstants.LEFT_EXTENSION_CONSTANTS),
                  new ExtensionIOTalonFX(V3_Horse_CV2_TR_IntakeConstants.RIGHT_EXTENSION_CONSTANTS),
                  () -> driver.getRightTriggerAxis());
          shooter =
              new V3_Horse_CV2_TRShooter(
                  new GenericFlywheelIOTalonFX(V3_Horse_CV2_TRShooterConstants.SHOOT_CONSTANTS),
                  new GenericHoodIOTalonFX(V3_Horse_CV2_TRShooterConstants.HOOD_CONSTANTS));
          vision =
              new Vision(
                  () -> FieldConstants.tagLayoutType.getLayout(),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V3_Horse_CV2_TRConstants.frontcamera),
                      V3_Horse_CV2_TRConstants.frontcamera,
                      V3_Horse_CV2_TRRobotState::getHeading,
                      drive::getMeasuredChassisVelocities,
                      V3_Horse_CV2_TRRobotState::getHeadingUpdateTimestamp,
                      List.of(V3_Horse_CV2_TRRobotState::addLocalizerVisionMeasurement),
                      List.of()));
          break;
        case V3_Horse_CV2_TR_SIM:
          drive =
              new SwerveDrive(
                  V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS,
                  new GyroIO() {},
                  new SwerveModuleIOSim(
                      V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS,
                      V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS.driveConfig.frontLeft()),
                  new SwerveModuleIOSim(
                      V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS,
                      V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS.driveConfig.frontRight()),
                  new SwerveModuleIOSim(
                      V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS,
                      V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS.driveConfig.backLeft()),
                  new SwerveModuleIOSim(
                      V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS,
                      V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS.driveConfig.backRight()),
                  V3_Horse_CV2_TRRobotState::getGlobalPose,
                  V3_Horse_CV2_TRRobotState::resetPose);
          rollerFloor =
              new V3_Horse_CV2_TR_RollerFloor(
                  new GenericRollerIOSim(
                      V3_Horse_CV2_TR_RollerFloorConstants.ROLLER_FLOOR_CONSTANTS));
          intake =
              new V3_Horse_CV2_TR_Intake(
                  new GenericRollerIOSim(
                      V3_Horse_CV2_TR_IntakeConstants.LEFT_INTAKE_ROLLER_CONSTANTS),
                  new GenericRollerIOSim(
                      V3_Horse_CV2_TR_IntakeConstants.RIGHT_INTAKE_ROLLER_CONSTANTS),
                  new GenericRollerIOSim(V3_Horse_CV2_TR_IntakeConstants.KICKER_ROLLER_CONSTANTS),
                  new ExtensionIOSim(V3_Horse_CV2_TR_IntakeConstants.LEFT_EXTENSION_CONSTANTS),
                  new ExtensionIOSim(V3_Horse_CV2_TR_IntakeConstants.RIGHT_EXTENSION_CONSTANTS),
                  () -> driver.getRightTriggerAxis());
          shooter =
              new V3_Horse_CV2_TRShooter(
                  new GenericFlywheelIOSim(V3_Horse_CV2_TRShooterConstants.SHOOT_CONSTANTS),
                  new GenericHoodIOSim(V3_Horse_CV2_TRShooterConstants.HOOD_CONSTANTS));
          vision = new Vision(() -> FieldConstants.tagLayoutType.getLayout());
          break;
        default:
      }
    }

    // Fall back to no-op IOs (e.g. in replay) so every subsystem always exists
    if (drive == null) {
      drive =
          new SwerveDrive(
              V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS,
              new GyroIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              V3_Horse_CV2_TRRobotState::getGlobalPose,
              V3_Horse_CV2_TRRobotState::resetPose);
    }
    if (rollerFloor == null) {
      rollerFloor = new V3_Horse_CV2_TR_RollerFloor(new GenericRollerIO() {});
    }
    if (intake == null) {
      intake =
          new V3_Horse_CV2_TR_Intake(
              new GenericRollerIO() {},
              new GenericRollerIO() {},
              new GenericRollerIO() {},
              new ExtensionIO() {},
              new ExtensionIO() {},
              () -> driver.getRightTriggerAxis());
    }
    if (shooter == null) {
      shooter = new V3_Horse_CV2_TRShooter(new GenericFlywheelIO() {}, new GenericHoodIO() {});
    }
    if (vision == null) {
      vision = new Vision(() -> FieldConstants.tagLayoutType.getLayout());
    }

    autoChooser = new LoggedNetworkChooser<>("Autonomous Modes");

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            V3_Horse_CV2_TRRobotState::getHeading));
    driver.a().onTrue(shooter.setHoodAngle(Rotation2d.ZERO));
    driver.b().onTrue(shooter.setHoodAngle(Rotation2d.fromDegrees(35)));
  }

  @Override
  public void robotPeriodic() {
    V3_Horse_CV2_TRRobotState.periodic(
        drive.getRawGyroRotation(), drive.getYawVelocity(), drive.getModulePositions(), drive);
  }

  @Override
  public Command getAutonomousCommand() {
    return shooter.hoodSysId();
  }
}
