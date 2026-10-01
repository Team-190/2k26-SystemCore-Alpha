package first.robot.v3_Horse_CV2_TR;

import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIO;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIOPigeon2;
import edu.wpi.team190.gompeilib.core.robot.RobotContainer;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIOSim;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.hood.GenericHoodIOSim;
import edu.wpi.team190.gompeilib.subsystems.generic.hood.GenericHoodIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOSim;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFX;
import first.robot.Constants;
import first.robot.RobotConfig;
import first.robot.util.CV2_input.XKeysInput;
import first.robot.util.CV2_input.XboxElite2Input;
import first.robot.v3_Horse_CV2_TR.commands.V3_Horse_CV2_TRDriveCommands;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TRRollerFloor;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TRRollerFloorConstants;
import first.robot.v3_Horse_CV2_TR.subsystems.shooter.V3_Horse_CV2_TRShooter;
import first.robot.v3_Horse_CV2_TR.subsystems.shooter.V3_Horse_CV2_TRShooterConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;

public class V3_Horse_CV2_TRRobotContainer implements RobotContainer {
  private GyroIO gyroIO;
  private V3_Horse_CV2_TRRollerFloor rollerFloor;
  private V3_Horse_CV2_TRShooter shooter;
//   private V3_Horse_CV2_Intake intake;
  private SwerveDrive drive;
  private final LoggedDashboardChooser<Command> autoChooser;

  private final XboxElite2Input driver = new XboxElite2Input(0);

  private final CommandGamepad driverController = new CommandGamepad(0);

  private final XKeysInput xkeys = new XKeysInput(1);

  private final CommandGamepad operatorController = new CommandGamepad(1);

  public V3_Horse_CV2_TRRobotContainer() {
    if (Constants.getMode() != RobotMode.REPLAY) {
      switch (RobotConfig.ROBOT) {
        case V3_Horse_CV2_TR:
          gyroIO =
              new GyroIOPigeon2(
                  V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS,
                  V3_Horse_CV2_TRRobotState::setHeadingUpdateTimestamp);
          rollerFloor =
              new V3_Horse_CV2_TRRollerFloor(
                  new GenericRollerIOTalonFX(
                      V3_Horse_CV2_TRRollerFloorConstants.ROLLER_FLOOR_CONSTANTS));
          shooter =
              new V3_Horse_CV2_TRShooter(
                  new GenericFlywheelIOTalonFX(V3_Horse_CV2_TRShooterConstants.SHOOT_CONSTANTS),
                  new GenericHoodIOTalonFX(V3_Horse_CV2_TRShooterConstants.HOOD_CONSTANTS));
          break;
        case V3_Horse_CV2_TR_SIM:
          rollerFloor =
              new V3_Horse_CV2_TRRollerFloor(
                  new GenericRollerIOSim(
                      V3_Horse_CV2_TRRollerFloorConstants.ROLLER_FLOOR_CONSTANTS));
          shooter =
              new V3_Horse_CV2_TRShooter(
                  new GenericFlywheelIOSim(V3_Horse_CV2_TRShooterConstants.SHOOT_CONSTANTS),
                  new GenericHoodIOSim(V3_Horse_CV2_TRShooterConstants.HOOD_CONSTANTS));
          break;
        default:
          break;
      }
    }
    autoChooser = new LoggedDashboardChooser<>("Autonomous Modes");
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
        .onTrue(Commands.none().withName("driver-leftBumper-true")); // Intake collect

    driver
        .rightBumper()
        .onTrue(Commands.none().withName("driver-rightBumper-true")); // Shoot when ready

    driver.rightTrigger().onTrue(Commands.none().withName("driver-rightTrigger-true")); // Agitate

    driver.dpadDown().onTrue(Commands.none().withName("driver-dpadDown-true")); // Reset heading

    driver
        .northFace() // Y Button
        .onTrue(Commands.none().withName("driver-Y-true")); // Shoot far

    driver
        .southFace() // A Button
        .onTrue(Commands.none().withName("driver-A-true")); // Shoot close

    driver
        .westFace() // X Button
        .onTrue(Commands.none().withName("driver-X-true")); // Shoot mid

    driver
        .eastFace() // B Button
        .onTrue(Commands.none().withName("driver-B-true")); // Intake stow
  }

  @Override
  public void robotPeriodic() {

    V3_Horse_CV2_TRRobotState.periodic(
        drive.getRawGyroRotation(), drive.getYawVelocity(), drive.getModulePositions(), drive);
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
