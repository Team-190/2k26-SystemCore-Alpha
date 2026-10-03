package first.robot.v3_Horse_CV2_TR;

import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIO;
import edu.wpi.team190.gompeilib.core.robot.RobotContainer;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOSim;
import edu.wpi.team190.gompeilib.subsystems.extension.ExtensionIO;
import edu.wpi.team190.gompeilib.subsystems.extension.ExtensionIOSim;
import edu.wpi.team190.gompeilib.subsystems.extension.ExtensionIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOSim;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFX;
import first.robot.Constants;
import first.robot.RobotConfig;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_TR_Intake;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_TR_IntakeConstants;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TR_RollerFloor;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TR_RollerFloorConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;
import org.wpilib.command2.Command;
import org.wpilib.command2.button.CommandNiDsXboxController;

public class V3_Horse_CV2_TRRobotContainer implements RobotContainer {
  private SwerveDrive drive;
  private V3_Horse_CV2_TR_RollerFloor rollerFloor;
  private V3_Horse_CV2_TR_Intake intake;
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
          break;
        case V3_Horse_CV2_TR_SIM:
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
          break;
        default:
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
      }
    }
    autoChooser = new LoggedNetworkChooser<>("Autonomous Modes");
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
