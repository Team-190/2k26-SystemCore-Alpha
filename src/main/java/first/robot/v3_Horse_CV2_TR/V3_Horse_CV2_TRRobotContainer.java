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
import first.robot.util.CV2_input.XKeysInput;
import first.robot.util.CV2_input.XboxElite2Input;
import first.robot.v3_Horse_CV2_TR.commands.V3_Horse_CV2_TRCompositeCommands;
import first.robot.v3_Horse_CV2_TR.commands.V3_Horse_CV2_TRDriveCommands;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_TR_Intake;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_TR_IntakeConstants;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_TR_IntakeConstants.ExtensionState;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_TR_IntakeConstants.RollerState;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TRRollerFloor;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TRRollerFloorConstants;
import first.robot.v3_Horse_CV2_TR.subsystems.shooter.V3_Horse_CV2_TRShooter;
import first.robot.v3_Horse_CV2_TR.subsystems.shooter.V3_Horse_CV2_TRShooterConstants;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;

public class V3_Horse_CV2_TRRobotContainer implements RobotContainer {
  private SwerveDrive drive;
  private V3_Horse_CV2_TRRollerFloor rollerFloor;
  private V3_Horse_CV2_TR_Intake intake;
  private V3_Horse_CV2_TRShooter shooter;
  private Vision vision;
  private final LoggedNetworkChooser<Command> autoChooser;

  private final XboxElite2Input driver = new XboxElite2Input(0);

  private final CommandGamepad driverController = new CommandGamepad(0);

  private final XKeysInput xkeys = new XKeysInput(1);

  private final CommandGamepad operatorController = new CommandGamepad(1);

  public V3_Horse_CV2_TRRobotContainer() {
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
              new V3_Horse_CV2_TRRollerFloor(
                  new GenericRollerIOTalonFX(
                      V3_Horse_CV2_TRRollerFloorConstants.ROLLER_FLOOR_CONSTANTS));
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
                  () -> driver.getRightTrigger());
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
              new V3_Horse_CV2_TRRollerFloor(
                  new GenericRollerIOSim(
                      V3_Horse_CV2_TRRollerFloorConstants.ROLLER_FLOOR_CONSTANTS));
          intake =
              new V3_Horse_CV2_TR_Intake(
                  new GenericRollerIOSim(
                      V3_Horse_CV2_TR_IntakeConstants.LEFT_INTAKE_ROLLER_CONSTANTS),
                  new GenericRollerIOSim(
                      V3_Horse_CV2_TR_IntakeConstants.RIGHT_INTAKE_ROLLER_CONSTANTS),
                  new GenericRollerIOSim(V3_Horse_CV2_TR_IntakeConstants.KICKER_ROLLER_CONSTANTS),
                  new ExtensionIOSim(V3_Horse_CV2_TR_IntakeConstants.LEFT_EXTENSION_CONSTANTS),
                  new ExtensionIOSim(V3_Horse_CV2_TR_IntakeConstants.RIGHT_EXTENSION_CONSTANTS),
                  () -> driver.getRightTrigger());
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
      rollerFloor = new V3_Horse_CV2_TRRollerFloor(new GenericRollerIO() {});
    }
    if (intake == null) {
      intake =
          new V3_Horse_CV2_TR_Intake(
              new GenericRollerIO() {},
              new GenericRollerIO() {},
              new GenericRollerIO() {},
              new ExtensionIO() {},
              new ExtensionIO() {},
              () -> driver.getRightTrigger());
    }
    if (shooter == null) {
      shooter = new V3_Horse_CV2_TRShooter(new GenericFlywheelIO() {}, new GenericHoodIO() {});
    }

    if (vision == null) {
      vision = new Vision(() -> FieldConstants.tagLayoutType.getLayout());
    }

    autoChooser = new LoggedNetworkChooser<>("Autonomous Modes");
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        V3_Horse_CV2_TRDriveCommands.joystickDriveRotationLock(
                drive,
                V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                V3_Horse_CV2_TRRobotState::getHeading,
                driver.rightTrigger(),
                () -> V3_Horse_CV2_TRRobotState.getRobotToHubAngle().getRadians(),
                () -> 0.0,
                driver.leftTrigger())
            .withName("joystickDriveRotationLock"));

    driver
        .leftTrigger()
        .onTrue(
            Commands.runOnce(
                    () ->
                        V3_Horse_CV2_TRDriveCommands.setLastCardinalDirection(
                            Math.round(
                                    V3_Horse_CV2_TRRobotState.getHeading().getRadians()
                                        / (Math.PI / 2.0))
                                * (Math.PI / 2.0)))
                .withName("cardinal-direction-set"));

    driver
        .leftBumper()
        .onTrue(intake.deploy().withName("driver-leftBumper-true")); 

    driver
        .rightBumper()
        .onTrue(
            V3_Horse_CV2_TRCompositeCommands.scoreOrFeedCommand(rollerFloor, shooter)
                .withName("driver-rightBumper-true"));

    driver.rightTrigger().onTrue(intake.setIntakeState(ExtensionState.AGITATE, RollerState.STOP).withName("driver-rightTrigger-true"));

    driver
        .dpadDown()
        .onTrue(
            V3_Horse_CV2_TRCompositeCommands.resetHeading(
                    drive,
                    V3_Horse_CV2_TRRobotState::resetPose,
                    () -> V3_Horse_CV2_TRRobotState.getGlobalPose().getTranslation())
                .withName("driver-dpadDown-true"));

    driver
        .faceUp()
        .onTrue(
            V3_Horse_CV2_TRCompositeCommands.farShotCommand(rollerFloor, shooter, intake)
                .withName("driver-Y-true"));

    driver
        .faceDown()
        .onTrue(
            V3_Horse_CV2_TRCompositeCommands.bumpShotCommand(rollerFloor, shooter, intake)
                .withName("driver-A-true"));

    driver
        .faceLeft()
        .onTrue(
            V3_Horse_CV2_TRCompositeCommands.trenchShotCommand(rollerFloor, shooter, intake)
                .withName("driver-X-true"));

    driver.faceRight().onTrue(intake.stowAndStop().withName("driver-B-true"));
  }

  @Override
  public void robotPeriodic() {
    V3_Horse_CV2_TRRobotState.periodic(
        drive.getRawGyroRotation(), drive.getYawVelocity(), drive.getModulePositions(), drive);
  }

  @Override
  public Command getAutonomousCommand() {
    return shooter.flywheelSysId();
  }
}
