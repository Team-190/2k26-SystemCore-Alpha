package first.robot.v3_Horse;

import static org.wpilib.units.Units.RadiansPerSecond;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.team190.gompeilib.io.gyro.interfaces.GyroIO;
import edu.wpi.team190.gompeilib.io.gyro.phoenix.GyroIOPigeon2;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.phoenix.MotorIOTalonFX;
import edu.wpi.team190.gompeilib.io.motor.sim.MotorIOSim;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixOdometryThread;
import edu.wpi.team190.gompeilib.robot.RobotContainer;
import edu.wpi.team190.gompeilib.robot.RobotMode;
import edu.wpi.team190.gompeilib.robot.mechanisms.extension.Extension;
import edu.wpi.team190.gompeilib.robot.mechanisms.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.robot.mechanisms.hood.GenericHood;
import edu.wpi.team190.gompeilib.robot.mechanisms.roller.GenericRoller;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDrive;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveModuleHardware;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands.DriveBehaviors;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands.DriveSticks;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands.SwerveDriveCommands;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.phoenix.SwerveModuleHardwareTalonFX;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.Vision;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.camera.CameraStaticLimelight;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.CameraIO;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.CameraIOLimelight;
import first.robot.Constants;
import first.robot.FieldConstants;
import first.robot.RobotConfig;
import first.robot.util.AllianceFlipUtil;
import first.robot.util.CV2_input.XKeysInput;
import first.robot.v3_Horse.commands.V3_HorseCompositeCommands;
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
import org.wpilib.math.geometry.Rotation2d;
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
          PhoenixOdometryThread odometryThread =
              PhoenixOdometryThread.getInstance(V3_HorseConstants.DRIVE_CONSTANTS);
          drive =
              new SwerveDrive(
                  V3_HorseConstants.DRIVE_CONSTANTS,
                  new GyroIOPigeon2(
                      V3_HorseConstants.DRIVE_CONSTANTS.gyroConfig(),
                      odometryThread,
                      V3_HorseRobotState::setHeadingUpdateTimestamp),
                  SwerveModuleHardwareTalonFX.create(
                      V3_HorseConstants.DRIVE_CONFIG.frontLeft(), odometryThread),
                  SwerveModuleHardwareTalonFX.create(
                      V3_HorseConstants.DRIVE_CONFIG.frontRight(), odometryThread),
                  SwerveModuleHardwareTalonFX.create(
                      V3_HorseConstants.DRIVE_CONFIG.backLeft(), odometryThread),
                  SwerveModuleHardwareTalonFX.create(
                      V3_HorseConstants.DRIVE_CONFIG.backRight(), odometryThread),
                  V3_HorseRobotState::getGlobalPose,
                  V3_HorseRobotState::resetPose);
          rollerFloor = new V3_HorseRollerFloor(MotorIOTalonFX.FACTORY);
          intake =
              new V3_HorseIntake(
                  MotorIOTalonFX.FACTORY,
                  MotorIOTalonFX.FACTORY,
                  MotorIOTalonFX.FACTORY,
                  MotorIOTalonFX.FACTORY,
                  () -> driver.getRightTriggerAxis());
          shooter =
              new V3_HorseShooter(
                  MotorIOTalonFX.FACTORY,
                  MotorIOTalonFX.FACTORY,
                  V3_HorseRobotState::isHopperRunning);
          vision =
              new Vision(
                  () -> FieldConstants.tagLayoutType.getLayout(),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V3_HorseConstants.LIMELIGHT_SHOOTER_CONFIG),
                      V3_HorseConstants.LIMELIGHT_SHOOTER_CONFIG,
                      V3_HorseRobotState::getHeading,
                      drive::getMeasuredChassisVelocities,
                      List.of(V3_HorseRobotState::addLocalizerVisionMeasurement),
                      List.of()),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V3_HorseConstants.LIMELIGHT_LEFT_CONFIG),
                      V3_HorseConstants.LIMELIGHT_LEFT_CONFIG,
                      V3_HorseRobotState::getHeading,
                      drive::getMeasuredChassisVelocities,
                      List.of(V3_HorseRobotState::addLocalizerVisionMeasurement),
                      List.of()));

          break;
        case V3_Horse_SIM:
          drive =
              new SwerveDrive(
                  V3_HorseConstants.DRIVE_CONSTANTS,
                  new GyroIO() {},
                  SwerveModuleHardware.sim(
                      V3_HorseConstants.DRIVE_CONFIG.frontLeft(), V3_HorseConstants.DRIVE_CONFIG),
                  SwerveModuleHardware.sim(
                      V3_HorseConstants.DRIVE_CONFIG.frontRight(), V3_HorseConstants.DRIVE_CONFIG),
                  SwerveModuleHardware.sim(
                      V3_HorseConstants.DRIVE_CONFIG.backLeft(), V3_HorseConstants.DRIVE_CONFIG),
                  SwerveModuleHardware.sim(
                      V3_HorseConstants.DRIVE_CONFIG.backRight(), V3_HorseConstants.DRIVE_CONFIG),
                  V3_HorseRobotState::getGlobalPose,
                  V3_HorseRobotState::resetPose);
          rollerFloor =
              new V3_HorseRollerFloor(
                  MotorIOSim.factory(
                      GenericRoller.simMechanism(
                          V3_HorseRollerFloorConstants.ROLLER_FLOOR_CONSTANTS)));
          intake =
              new V3_HorseIntake(
                  MotorIOSim.factory(
                      GenericRoller.simMechanism(V3_HorseIntakeConstants.INTAKE_ROLLER_CONSTANTS)),
                  MotorIOSim.factory(
                      GenericRoller.simMechanism(V3_HorseIntakeConstants.KICKER_ROLLER_CONSTANTS)),
                  MotorIOSim.factory(
                      Extension.simMechanism(V3_HorseIntakeConstants.LEFT_EXTENSION_CONSTANTS)),
                  MotorIOSim.factory(
                      Extension.simMechanism(V3_HorseIntakeConstants.RIGHT_EXTENSION_CONSTANTS)),
                  () -> driver.getRightTriggerAxis());
          shooter =
              new V3_HorseShooter(
                  MotorIOSim.factory(
                      GenericFlywheel.simMechanism(V3_HorseShooterConstants.SHOOT_CONSTANTS)),
                  MotorIOSim.factory(
                      GenericHood.simMechanism(V3_HorseShooterConstants.HOOD_CONSTANTS)),
                  V3_HorseRobotState::isHopperRunning);
          vision =
              new Vision(
                  () -> FieldConstants.tagLayoutType.getLayout(),
                  new CameraStaticLimelight(
                      new CameraIO() {},
                      V3_HorseConstants.LIMELIGHT_SHOOTER_CONFIG,
                      V3_HorseRobotState::getHeading,
                      drive::getMeasuredChassisVelocities,
                      List.of(V3_HorseRobotState::addLocalizerVisionMeasurement),
                      List.of()),
                  new CameraStaticLimelight(
                      new CameraIO() {},
                      V3_HorseConstants.LIMELIGHT_LEFT_CONFIG,
                      V3_HorseRobotState::getHeading,
                      drive::getMeasuredChassisVelocities,
                      List.of(V3_HorseRobotState::addLocalizerVisionMeasurement),
                      List.of()));
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
              SwerveModuleHardware.REPLAY,
              SwerveModuleHardware.REPLAY,
              SwerveModuleHardware.REPLAY,
              SwerveModuleHardware.REPLAY,
              V3_HorseRobotState::getGlobalPose,
              V3_HorseRobotState::resetPose);
    }
    if (rollerFloor == null) {
      rollerFloor = new V3_HorseRollerFloor(MotorIO.REPLAY_FACTORY);
    }
    if (intake == null) {
      intake =
          new V3_HorseIntake(
              MotorIO.REPLAY_FACTORY,
              MotorIO.REPLAY_FACTORY,
              MotorIO.REPLAY_FACTORY,
              MotorIO.REPLAY_FACTORY,
              () -> driver.getRightTriggerAxis());
    }
    if (shooter == null) {
      shooter =
          new V3_HorseShooter(
              MotorIO.REPLAY_FACTORY, MotorIO.REPLAY_FACTORY, V3_HorseRobotState::isHopperRunning);
    }

    if (vision == null) {
      vision =
          new Vision(
              () -> FieldConstants.tagLayoutType.getLayout(),
              new CameraStaticLimelight(
                  new CameraIO() {},
                  V3_HorseConstants.LIMELIGHT_SHOOTER_CONFIG,
                  V3_HorseRobotState::getHeading,
                  drive::getMeasuredChassisVelocities,
                  List.of(V3_HorseRobotState::addLocalizerVisionMeasurement),
                  List.of()),
              new CameraStaticLimelight(
                  new CameraIO() {},
                  V3_HorseConstants.LIMELIGHT_LEFT_CONFIG,
                  V3_HorseRobotState::getHeading,
                  drive::getMeasuredChassisVelocities,
                  List.of(V3_HorseRobotState::addLocalizerVisionMeasurement),
                  List.of()));
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

    autoChooser.addDefault("Do Nothing", Commands.none());

    // Characterizations of the closed-loop mechanisms print each one's plant and gains when they
    // finish
    autoChooser.add(
        "Characterize: Drive",
        SwerveDriveCommands.feedforwardCharacterization(
            drive, V3_HorseConstants.DRIVE_CHARACTERIZATION));
    autoChooser.add("Characterize: Hood", shooter.characterizeHood());
    autoChooser.add("Characterize: Flywheel (Torque)", shooter.characterizeFlywheel());
    autoChooser.add("Characterize: Flywheel (Voltage)", shooter.characterizeFlywheelVoltage());
    autoChooser.add("Characterize: Extensions", intake.characterizeExtensions());
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        SwerveDriveCommands.joystickDrive(
                drive,
                V3_HorseConstants.DRIVE_CONSTANTS,
                new DriveSticks(
                    () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()),
                V3_HorseRobotState::getHeading)
            .withDriverPerspective(() -> AllianceFlipUtil.apply(Rotation2d.ZERO))
            // Aim at hub or feed point
            .with(
                driver.rightBumper(),
                DriveBehaviors.faceHeading(
                    V3_HorseConstants.DRIVE_CONSTANTS, V3_HorseRobotState::getAimAngle))
            .with(
                driver.leftTrigger(),
                DriveBehaviors.faceHeading(
                    V3_HorseConstants.DRIVE_CONSTANTS, V3_HorseRobotState::getCardinalDirection))
            .build()
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
                    V3_HorseRobotState::resetPose,
                    () -> V3_HorseRobotState.getGlobalPose().getTranslation())
                .withName("driver-dpadDown-true"));

    driver
        .y()
        .whileTrue(
            V3_HorseCompositeCommands.fixedShotCommand(
                    rollerFloor, shooter, intake, V3_HorseRobotState.FixedShots.CORNER)
                .withName("driver-Y-true"))
        .onFalse(stopShooter("driver-Y-false"));

    driver
        .a()
        .whileTrue(
            V3_HorseCompositeCommands.fixedShotCommand(
                    rollerFloor, shooter, intake, V3_HorseRobotState.FixedShots.FRONT_TOWER)
                .withName("driver-A-true"))
        .onFalse(stopShooter("driver-A-false"));

    driver
        .x()
        .whileTrue(
            V3_HorseCompositeCommands.fixedShotCommand(
                    rollerFloor, shooter, intake, V3_HorseRobotState.FixedShots.SIDE_TOWER)
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
        V3_HorseMechanism3D.getPoses(intake.getExtensionPosition(), shooter.getHoodAngle()));

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
