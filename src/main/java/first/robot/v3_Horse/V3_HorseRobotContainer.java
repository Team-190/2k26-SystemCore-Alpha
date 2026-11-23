package first.robot.v3_Horse;

import static org.wpilib.units.Units.RadiansPerSecond;

import com.pathplanner.lib.auto.NamedCommands;
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
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIOTalonFXSim;
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
import first.robot.util.AllianceFlipUtil;
import first.robot.util.CV2_input.XKeysInput;
import first.robot.v3_Horse.commands.V3_HorseCompositeCommands;
import first.robot.v3_Horse.commands.V3_HorseDriveCommands;
import first.robot.v3_Horse.commands.autonomous.V3_HorseAutoBackUpShoot;
import first.robot.v3_Horse.commands.autonomous.V3_HorseAutoDoubleDip;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntake;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntakeConstants;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntakeConstants.ExtensionState;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntakeConstants.RollerState;
import first.robot.v3_Horse.subsystems.rollerfloor.V3_HorseRollerFloor;
import first.robot.v3_Horse.subsystems.rollerfloor.V3_HorseRollerFloorConstants;
import first.robot.v3_Horse.subsystems.rollerfloor.V3_HorseRollerFloorConstants.RollerFloorState;
import first.robot.v3_Horse.subsystems.shooter.V3_HorseShooter;
import first.robot.v3_Horse.subsystems.shooter.V3_HorseShooterConstants;
import first.robot.v3_Horse.subsystems.shooter.V3_HorseShooterConstants.ShooterGoal;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringSubscriber;

public class V3_HorseRobotContainer implements RobotContainer {
  private SwerveDrive drive;
  private V3_HorseRollerFloor rollerFloor;
  private V3_HorseIntake intake;
  private V3_HorseShooter shooter;
  private Vision vision;
  private final LoggedNetworkChooser<Command> autoChooser;

  private final StringSubscriber selectedAutoName =
      NetworkTableInstance.getDefault()
          .getStringTopic("/Autonomous Modes/selected/value")
          .subscribe("");

  private final Map<String, Pose2d[]> autoTrajectories =
      Map.of(
          "Back up and shoot",
          V3_HorseAutoBackUpShoot.getTrajectory(),
          "Left Cheesy",
          V3_HorseAutoDoubleDip.getTrajectory("DOUBLE_DIP_1", "DOUBLE_DIP_2_CHEESY", false),
          "Right Cheesy",
          V3_HorseAutoDoubleDip.getTrajectory("DOUBLE_DIP_1", "DOUBLE_DIP_2_CHEESY", true),
          "Left Outliers",
          V3_HorseAutoDoubleDip.getTrajectory("DOUBLE_DIP_1", "DOUBLE_DIP_2_OUTLIERS", false),
          "Right Outliers",
          V3_HorseAutoDoubleDip.getTrajectory("DOUBLE_DIP_1", "DOUBLE_DIP_2_OUTLIERS", true));

  private String lastPublishedAuto = null;
  private boolean lastPublishedFlipped = false;

  private final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);

  @SuppressWarnings("unused")
  private final XKeysInput xkeys = new XKeysInput(1);

  private final CommandNiDsXboxController operator = new CommandNiDsXboxController(2);

  public V3_HorseRobotContainer() {
    if (Constants.getMode() != RobotMode.REPLAY) {
      switch (RobotConfig.ROBOT) {
        case V3_Horse:
          drive =
              new SwerveDrive(
                  V3_HorseConstants.DRIVE_CONSTANTS,
                  new GyroIOPigeon2(
                      V3_HorseConstants.DRIVE_CONSTANTS,
                      V3_HorseRobotState::setHeadingUpdateTimestamp),
                  new SwerveModuleIOTalonFX(
                      V3_HorseConstants.DRIVE_CONSTANTS,
                      V3_HorseConstants.DRIVE_CONSTANTS.driveConfig.frontLeft()),
                  new SwerveModuleIOTalonFX(
                      V3_HorseConstants.DRIVE_CONSTANTS,
                      V3_HorseConstants.DRIVE_CONSTANTS.driveConfig.frontRight()),
                  new SwerveModuleIOTalonFX(
                      V3_HorseConstants.DRIVE_CONSTANTS,
                      V3_HorseConstants.DRIVE_CONSTANTS.driveConfig.backLeft()),
                  new SwerveModuleIOTalonFX(
                      V3_HorseConstants.DRIVE_CONSTANTS,
                      V3_HorseConstants.DRIVE_CONSTANTS.driveConfig.backRight()),
                  V3_HorseRobotState::getGlobalPose,
                  V3_HorseRobotState::resetPose);
          rollerFloor =
              new V3_HorseRollerFloor(
                  new GenericRollerIOTalonFX(V3_HorseRollerFloorConstants.ROLLER_FLOOR_CONSTANTS));
          intake =
              new V3_HorseIntake(
                  new GenericRollerIOTalonFX(V3_HorseIntakeConstants.LEFT_INTAKE_ROLLER_CONSTANTS),
                  new GenericRollerIOTalonFX(V3_HorseIntakeConstants.RIGHT_INTAKE_ROLLER_CONSTANTS),
                  new GenericRollerIOTalonFX(V3_HorseIntakeConstants.KICKER_ROLLER_CONSTANTS),
                  new ExtensionIOTalonFX(V3_HorseIntakeConstants.LEFT_EXTENSION_CONSTANTS),
                  new ExtensionIOTalonFX(V3_HorseIntakeConstants.RIGHT_EXTENSION_CONSTANTS),
                  () -> driver.getRightTriggerAxis());
          shooter =
              new V3_HorseShooter(
                  new GenericFlywheelIOTalonFX(V3_HorseShooterConstants.SHOOT_CONSTANTS),
                  new GenericHoodIOTalonFX(V3_HorseShooterConstants.HOOD_CONSTANTS),
                  V3_HorseRobotState::isHopperRunning);
          vision =
              new Vision(
                  () -> FieldConstants.tagLayoutType.getLayout(),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V3_HorseConstants.LIMELIGHT_SHOOTER_CONFIG),
                      V3_HorseConstants.LIMELIGHT_SHOOTER_CONFIG,
                      V3_HorseRobotState::getHeading,
                      drive::getMeasuredChassisVelocities,
                      V3_HorseRobotState::getHeadingUpdateTimestamp,
                      List.of(V3_HorseRobotState::addLocalizerVisionMeasurement),
                      List.of()),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V3_HorseConstants.LIMELIGHT_LEFT_CONFIG),
                      V3_HorseConstants.LIMELIGHT_LEFT_CONFIG,
                      V3_HorseRobotState::getHeading,
                      drive::getMeasuredChassisVelocities,
                      V3_HorseRobotState::getHeadingUpdateTimestamp,
                      List.of(V3_HorseRobotState::addLocalizerVisionMeasurement),
                      List.of()));

          break;
        case V3_Horse_SIM:
          drive =
              new SwerveDrive(
                  V3_HorseConstants.DRIVE_CONSTANTS,
                  new GyroIO() {},
                  new SwerveModuleIOSim(
                      V3_HorseConstants.DRIVE_CONSTANTS,
                      V3_HorseConstants.DRIVE_CONSTANTS.driveConfig.frontLeft()),
                  new SwerveModuleIOSim(
                      V3_HorseConstants.DRIVE_CONSTANTS,
                      V3_HorseConstants.DRIVE_CONSTANTS.driveConfig.frontRight()),
                  new SwerveModuleIOSim(
                      V3_HorseConstants.DRIVE_CONSTANTS,
                      V3_HorseConstants.DRIVE_CONSTANTS.driveConfig.backLeft()),
                  new SwerveModuleIOSim(
                      V3_HorseConstants.DRIVE_CONSTANTS,
                      V3_HorseConstants.DRIVE_CONSTANTS.driveConfig.backRight()),
                  V3_HorseRobotState::getGlobalPose,
                  V3_HorseRobotState::resetPose);
          rollerFloor =
              new V3_HorseRollerFloor(
                  new GenericRollerIOSim(V3_HorseRollerFloorConstants.ROLLER_FLOOR_CONSTANTS));
          intake =
              new V3_HorseIntake(
                  new GenericRollerIOSim(V3_HorseIntakeConstants.LEFT_INTAKE_ROLLER_CONSTANTS),
                  new GenericRollerIOSim(V3_HorseIntakeConstants.RIGHT_INTAKE_ROLLER_CONSTANTS),
                  new GenericRollerIOSim(V3_HorseIntakeConstants.KICKER_ROLLER_CONSTANTS),
                  new ExtensionIOSim(V3_HorseIntakeConstants.LEFT_EXTENSION_CONSTANTS),
                  new ExtensionIOSim(V3_HorseIntakeConstants.RIGHT_EXTENSION_CONSTANTS),
                  () -> driver.getRightTriggerAxis());
          shooter =
              new V3_HorseShooter(
                  new GenericFlywheelIOTalonFXSim(V3_HorseShooterConstants.SHOOT_CONSTANTS),
                  new GenericHoodIOSim(V3_HorseShooterConstants.HOOD_CONSTANTS),
                  V3_HorseRobotState::isHopperRunning);
          vision = new Vision(() -> FieldConstants.tagLayoutType.getLayout());
          break;
        default:
      }
    }

    // Fall back to no-op IOs (e.g. in replay) so every subsystem always exists
    if (drive == null) {
      drive =
          new SwerveDrive(
              V3_HorseConstants.DRIVE_CONSTANTS,
              new GyroIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              V3_HorseRobotState::getGlobalPose,
              V3_HorseRobotState::resetPose);
    }
    if (rollerFloor == null) {
      rollerFloor = new V3_HorseRollerFloor(new GenericRollerIO() {});
    }
    if (intake == null) {
      intake =
          new V3_HorseIntake(
              new GenericRollerIO() {},
              new GenericRollerIO() {},
              new GenericRollerIO() {},
              new ExtensionIO() {},
              new ExtensionIO() {},
              () -> driver.getRightTriggerAxis());
    }
    if (shooter == null) {
      shooter =
          new V3_HorseShooter(
              new GenericFlywheelIO() {},
              new GenericHoodIO() {},
              V3_HorseRobotState::isHopperRunning);
    }

    if (vision == null) {
      vision = new Vision(() -> FieldConstants.tagLayoutType.getLayout());
    }

    autoChooser = new LoggedNetworkChooser<>("Autonomous Modes");
    configureButtonBindings();
    configureAutos();
  }

  private void configureAutos() {

    NamedCommands.registerCommand("INTAKE", V3_HorseCompositeCommands.intakeCollect(intake));
    NamedCommands.registerCommand("STOP_INTAKE", intake.setRollerState(RollerState.IDLE));

    autoChooser.add(
        "Back up and shoot",
        V3_HorseAutoBackUpShoot.getAutoRoutine(drive, shooter, rollerFloor, intake));

    autoChooser.add(
        "Left Cheesy",
        V3_HorseAutoDoubleDip.getAutoRoutine(
            drive,
            shooter,
            rollerFloor,
            intake,
            "DOUBLE_DIP_1",
            "DOUBLE_DIP_2_CHEESY",
            false,
            4,
            5));

    autoChooser.add(
        "Right Cheesy",
        V3_HorseAutoDoubleDip.getAutoRoutine(
            drive,
            shooter,
            rollerFloor,
            intake,
            "DOUBLE_DIP_1",
            "DOUBLE_DIP_2_CHEESY",
            true,
            4,
            5));

    autoChooser.add(
        "Left Outliers",
        V3_HorseAutoDoubleDip.getAutoRoutine(
            drive,
            shooter,
            rollerFloor,
            intake,
            "DOUBLE_DIP_1",
            "DOUBLE_DIP_2_OUTLIERS",
            false,
            4,
            5));

    autoChooser.add(
        "Right Outliers",
        V3_HorseAutoDoubleDip.getAutoRoutine(
            drive,
            shooter,
            rollerFloor,
            intake,
            "DOUBLE_DIP_1",
            "DOUBLE_DIP_2_OUTLIERS",
            true,
            4,
            5));

    autoChooser.add("Shooter Flywheel Torque SysId", shooter.flywheelSysId());

    autoChooser.addDefault("Do Nothing", Commands.none());
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        V3_HorseDriveCommands.joystickDriveRotationLock(
                drive,
                V3_HorseConstants.DRIVE_CONSTANTS,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                V3_HorseRobotState::getHeading,
                () -> driver.rightBumper().getAsBoolean(), // Aim at hub or feed point
                () -> V3_HorseRobotState.getAimAngle().getRadians(),
                () -> 0.0,
                driver.leftTrigger(),
                () -> V3_HorseRobotState.getCardinalDirection())
            .withName("joystickDriveRotationLock"));

    driver.leftTrigger().onTrue(Commands.runOnce(() -> {}).withName("cardinal-direction-set"));

    driver
        .leftBumper()
        .whileTrue(
            V3_HorseCompositeCommands.intakeCollect(intake).withName("driver-leftBumper-true"))
        .onFalse(intake.setRollerState(RollerState.IDLE).withName("driver-leftBumper-false"));

    driver
        .rightBumper()
        .whileTrue(
            V3_HorseCompositeCommands.scoreOrFeedCommand(rollerFloor, shooter, intake)
                .withName("driver-rightBumper-true"))
        .onFalse(stopShooter("driver-rightBumper-false"));

    driver
        .rightTrigger(0.0)
        .onTrue(
            intake
                .setExtensionState(ExtensionState.MANUAL_EXTEND)
                .withName("driver-rightTrigger-true"))
        .onFalse(intake.releaseManualExtend().withName("driver-rightTrigger-false"));

    driver
        .getHID()
        .povDown()
        .onTrue(
            V3_HorseCompositeCommands.resetHeading(
                    drive,
                    V3_HorseRobotState::resetPose,
                    () -> V3_HorseRobotState.getGlobalPose().getTranslation())
                .withName("driver-dpadDown-true"));

    driver
        .y()
        .whileTrue(
            V3_HorseCompositeCommands.fixedShotCommand(
                    drive, rollerFloor, shooter, intake, V3_HorseRobotState.FixedShots.CORNER)
                .withName("driver-Y-true"))
        .onFalse(stopShooter("driver-Y-false"));

    driver
        .a()
        .whileTrue(
            V3_HorseCompositeCommands.fixedShotCommand(
                    drive, rollerFloor, shooter, intake, V3_HorseRobotState.FixedShots.FRONT_TOWER)
                .withName("driver-A-true"))
        .onFalse(stopShooter("driver-A-false"));

    driver
        .x()
        .whileTrue(
            V3_HorseCompositeCommands.fixedShotCommand(
                    drive, rollerFloor, shooter, intake, V3_HorseRobotState.FixedShots.SIDE_TOWER)
                .withName("driver-X-true"))
        .onFalse(stopShooter("driver-X-false"));

    driver
        .start()
        .onTrue(
            shooter
                .setFlywheelVelocity(RadiansPerSecond.of(165))
                .withName("driver-top-right-true"));

    driver.b().onTrue(V3_HorseCompositeCommands.intakeStow(intake).withName("driver-B-true"));

    operator
        .leftBumper()
        .onTrue(V3_HorseCompositeCommands.intakeStow(intake).withName("operator-leftBumper-true"));

    operator
        .rightBumper()
        .whileTrue(
            V3_HorseCompositeCommands.intakeCollect(intake).withName("operator-rightBumper-true"))
        .onFalse(intake.setRollerState(RollerState.IDLE).withName("operator-leftBumper-false"));

    operator
        .leftTrigger()
        .whileTrue(intake.intakeRollers().withName("operator-leftTrigger-true"))
        .onFalse(intake.setRollerState(RollerState.IDLE).withName("operator-leftTrigger-false"));

    operator
        .rightTrigger()
        .whileTrue(intake.extakeRollers().withName("operator-rightTrigger-true"))
        .onFalse(intake.setRollerState(RollerState.IDLE).withName("operator-leftTrigger-false"));
    operator
        .getHID()
        .povUp()
        .whileTrue(
            Commands.runOnce(
                    () ->
                        rollerFloor.setOverrideRollerFloorVoltage(
                            V3_HorseRollerFloorConstants.ROLLER_FLOOR_SLOW_VOLTAGE))
                .andThen(rollerFloor.setState(RollerFloorState.OVERRIDE))
                .withName("operator-dPadUp-true"))
        .onFalse(rollerFloor.setState(RollerFloorState.STOP).withName("operator-dPadUp-false"));

    operator
        .getHID()
        .povDown()
        .whileTrue(
            Commands.runOnce(
                    () ->
                        rollerFloor.setOverrideRollerFloorVoltage(
                            V3_HorseRollerFloorConstants.ROLLER_FLOOR_RUN_VOLTAGE.times(-1)))
                .andThen(rollerFloor.setState(RollerFloorState.OVERRIDE))
                .withName("operator-dPadDown-true"))
        .onFalse(rollerFloor.setState(RollerFloorState.STOP).withName("operator-dPadDown-false"));

    operator.getHID().povLeft().onTrue(shooter.resetHoodZero().withName("operator-dPadLeft-true"));

    operator
        .getHID()
        .povRight()
        .onTrue(intake.zeroExtensions().withName("operator-dPadRight-true"));

    operator.y().onTrue(shooter.incrementFlywheelVelocity().withName("operator-Y-true"));
    operator.x().onTrue(shooter.decrementFlywheelVelocity().withName("operator-X-true"));
    operator.b().onTrue(shooter.incrementHoodAngle().withName("operator-B-true"));
    operator.a().onTrue(shooter.decrementHoodAngle().withName("operator-A-true"));

    operator.back().onTrue(shooter.setGoal(ShooterGoal.STOW).withName("operator-back-true"));
    operator.start().onTrue(intake.resetIntakeZeroPosition().withName("operator-start-true"));

    operator
        .leftStick()
        .whileTrue(intake.slowMoveIn().withName("operator-leftStick-true"))
        .onFalse(intake.stopExtensions().withName("operator-leftStick-false"));
    operator
        .rightStick()
        .whileTrue(intake.slowMoveOut().withName("operator-rightStick-true"))
        .onFalse(intake.stopExtensions().withName("operator-rightStick-false"));
  }

  private Command stopShooter(String name) {
    return V3_HorseCompositeCommands.stopShooter(rollerFloor, shooter).withName(name);
  }

  @Override
  public void robotPeriodic() {
    V3_HorseRobotState.periodic(
        drive.getRawGyroRotation(), drive.getYawVelocity(), drive.getModulePositions(), drive);

    Logger.recordOutput(
        "Mechanism3D",
        V3_HorseMechanism3D.getPoses(
            intake.getExtensionPosition(), shooter.getHoodAngle().getMeasure()));

    String autoName = selectedAutoName.get();
    boolean flipped = AllianceFlipUtil.shouldFlip();

    if (!autoName.equals(lastPublishedAuto) || flipped != lastPublishedFlipped) {
      Pose2d[] trajectory = autoTrajectories.getOrDefault(autoName, new Pose2d[0]);
      if (flipped) {
        trajectory = Arrays.stream(trajectory).map(AllianceFlipUtil::apply).toArray(Pose2d[]::new);
      }
      V3_HorseRobotState.setAutoTrajectory(trajectory);

      lastPublishedAuto = autoName;
      lastPublishedFlipped = flipped;
    }
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
