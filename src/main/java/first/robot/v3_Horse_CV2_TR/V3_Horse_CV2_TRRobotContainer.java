package first.robot.v3_Horse_CV2_TR;

import edu.wpi.team190.gompeilib.io.gyro.interfaces.GyroIO;
import edu.wpi.team190.gompeilib.io.gyro.phoenix.GyroIOPigeon2;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.phoenix.MotorIOTalonFX;
import edu.wpi.team190.gompeilib.io.motor.sim.MotorIOSim;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixOdometryThread;
import edu.wpi.team190.gompeilib.robot.RobotContainer;
import edu.wpi.team190.gompeilib.robot.RobotMode;
import edu.wpi.team190.gompeilib.subsystems.drive.swerve.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drive.swerve.SwerveModuleHardware;
import edu.wpi.team190.gompeilib.subsystems.drive.swerve.phoenix.SwerveModuleHardwareTalonFX;
import edu.wpi.team190.gompeilib.subsystems.extension.Extension;
import edu.wpi.team190.gompeilib.subsystems.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.subsystems.hood.GenericHoodIO;
import edu.wpi.team190.gompeilib.subsystems.hood.GenericHoodIOSim;
import edu.wpi.team190.gompeilib.subsystems.hood.GenericHoodIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.vision.Vision;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraStaticLimelight;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOLimelight;
import first.robot.Constants;
import first.robot.FieldConstants;
import first.robot.RobotConfig;
import first.robot.v3_Horse_CV2_TR.commands.V3_Horse_CV2_TRCompositeCommands;
import first.robot.v3_Horse_CV2_TR.commands.V3_Horse_CV2_TRDriveCommands;
import first.robot.v3_Horse_CV2_TR.commands.autonomous.V3_Horse_CV2_TRAutoBackUpShoot;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_TR_Intake;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_TR_IntakeConstants;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_TR_IntakeConstants.ExtensionState;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_TR_IntakeConstants.RollerState;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TRRollerFloor;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TRRollerFloorConstants;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TRRollerFloorConstants.RollerFloorState;
import first.robot.v3_Horse_CV2_TR.subsystems.shooter.V3_Horse_CV2_TRShooter;
import first.robot.v3_Horse_CV2_TR.subsystems.shooter.V3_Horse_CV2_TRShooterConstants;
import first.robot.v3_Horse_CV2_TR.subsystems.shooter.V3_Horse_CV2_TRShooterConstants.ShooterGoal;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandNiDsXboxController;

public class V3_Horse_CV2_TRRobotContainer implements RobotContainer {
  private SwerveDrive drive;
  private V3_Horse_CV2_TRRollerFloor rollerFloor;
  private V3_Horse_CV2_TR_Intake intake;
  private V3_Horse_CV2_TRShooter shooter;
  private Vision vision;
  private final LoggedNetworkChooser<Command> autoChooser;

  private final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);

  private final CommandNiDsXboxController operator = new CommandNiDsXboxController(2);

  public V3_Horse_CV2_TRRobotContainer() {
    if (Constants.getMode() != RobotMode.REPLAY) {
      switch (RobotConfig.ROBOT) {
        case V3_Horse_CV2_TR:
          PhoenixOdometryThread odometryThread =
              PhoenixOdometryThread.getInstance(V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS);
          drive =
              new SwerveDrive(
                  V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS,
                  new GyroIOPigeon2(
                      V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS.gyroConfig(),
                      odometryThread,
                      V3_Horse_CV2_TRRobotState::setHeadingUpdateTimestamp),
                  SwerveModuleHardwareTalonFX.create(
                      V3_Horse_CV2_TRConstants.DRIVE_CONFIG.frontLeft(), odometryThread),
                  SwerveModuleHardwareTalonFX.create(
                      V3_Horse_CV2_TRConstants.DRIVE_CONFIG.frontRight(), odometryThread),
                  SwerveModuleHardwareTalonFX.create(
                      V3_Horse_CV2_TRConstants.DRIVE_CONFIG.backLeft(), odometryThread),
                  SwerveModuleHardwareTalonFX.create(
                      V3_Horse_CV2_TRConstants.DRIVE_CONFIG.backRight(), odometryThread),
                  V3_Horse_CV2_TRRobotState::getGlobalPose,
                  V3_Horse_CV2_TRRobotState::resetPose);
          rollerFloor = new V3_Horse_CV2_TRRollerFloor(MotorIOTalonFX.FACTORY);
          intake =
              new V3_Horse_CV2_TR_Intake(
                  MotorIOTalonFX.FACTORY,
                  MotorIOTalonFX.FACTORY,
                  MotorIOTalonFX.FACTORY,
                  MotorIOTalonFX.FACTORY,
                  () -> driver.getRightTriggerAxis());
          shooter =
              new V3_Horse_CV2_TRShooter(
                  MotorIOTalonFX.FACTORY,
                  new GenericHoodIOTalonFX(V3_Horse_CV2_TRShooterConstants.HOOD_CONSTANTS));
          vision =
              new Vision(
                  () -> FieldConstants.tagLayoutType.getLayout(),
                  new CameraStaticLimelight(
                      new CameraIOLimelight(V3_Horse_CV2_TRConstants.LIMELIGHT_SHOOTER_CONFIG),
                      V3_Horse_CV2_TRConstants.LIMELIGHT_SHOOTER_CONFIG,
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
                  SwerveModuleHardware.sim(
                      V3_Horse_CV2_TRConstants.DRIVE_CONFIG.frontLeft(),
                      V3_Horse_CV2_TRConstants.DRIVE_CONFIG),
                  SwerveModuleHardware.sim(
                      V3_Horse_CV2_TRConstants.DRIVE_CONFIG.frontRight(),
                      V3_Horse_CV2_TRConstants.DRIVE_CONFIG),
                  SwerveModuleHardware.sim(
                      V3_Horse_CV2_TRConstants.DRIVE_CONFIG.backLeft(),
                      V3_Horse_CV2_TRConstants.DRIVE_CONFIG),
                  SwerveModuleHardware.sim(
                      V3_Horse_CV2_TRConstants.DRIVE_CONFIG.backRight(),
                      V3_Horse_CV2_TRConstants.DRIVE_CONFIG),
                  V3_Horse_CV2_TRRobotState::getGlobalPose,
                  V3_Horse_CV2_TRRobotState::resetPose);
          rollerFloor =
              new V3_Horse_CV2_TRRollerFloor(
                  MotorIOSim.factory(
                      GenericRoller.simMechanism(
                          V3_Horse_CV2_TRRollerFloorConstants.ROLLER_FLOOR_CONSTANTS)));
          intake =
              new V3_Horse_CV2_TR_Intake(
                  MotorIOSim.factory(
                      GenericRoller.simMechanism(
                          V3_Horse_CV2_TR_IntakeConstants.INTAKE_ROLLER_CONSTANTS)),
                  MotorIOSim.factory(
                      GenericRoller.simMechanism(
                          V3_Horse_CV2_TR_IntakeConstants.KICKER_ROLLER_CONSTANTS)),
                  MotorIOSim.factory(
                      Extension.simMechanism(
                          V3_Horse_CV2_TR_IntakeConstants.LEFT_EXTENSION_CONSTANTS)),
                  MotorIOSim.factory(
                      Extension.simMechanism(
                          V3_Horse_CV2_TR_IntakeConstants.RIGHT_EXTENSION_CONSTANTS)),
                  () -> driver.getRightTriggerAxis());
          shooter =
              new V3_Horse_CV2_TRShooter(
                  MotorIOSim.factory(
                      GenericFlywheel.simMechanism(
                          V3_Horse_CV2_TRShooterConstants.SHOOT_CONSTANTS)),
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
              SwerveModuleHardware.REPLAY,
              SwerveModuleHardware.REPLAY,
              SwerveModuleHardware.REPLAY,
              SwerveModuleHardware.REPLAY,
              V3_Horse_CV2_TRRobotState::getGlobalPose,
              V3_Horse_CV2_TRRobotState::resetPose);
    }
    if (rollerFloor == null) {
      rollerFloor = new V3_Horse_CV2_TRRollerFloor(MotorIO.REPLAY_FACTORY);
    }
    if (intake == null) {
      intake =
          new V3_Horse_CV2_TR_Intake(
              MotorIO.REPLAY_FACTORY,
              MotorIO.REPLAY_FACTORY,
              MotorIO.REPLAY_FACTORY,
              MotorIO.REPLAY_FACTORY,
              () -> driver.getRightTriggerAxis());
    }
    if (shooter == null) {
      shooter = new V3_Horse_CV2_TRShooter(MotorIO.REPLAY_FACTORY, new GenericHoodIO() {});
    }

    if (vision == null) {
      vision = new Vision(() -> FieldConstants.tagLayoutType.getLayout());
    }

    autoChooser = new LoggedNetworkChooser<>("Autonomous Modes");
    configureButtonBindings();
    configureAutos();
  }

  private void configureAutos() {
    autoChooser.add(
        "Back up and shoot",
        V3_Horse_CV2_TRAutoBackUpShoot.getAutoRoutine(drive, shooter, rollerFloor));

    autoChooser.addDefault("Do Nothing", Commands.none());
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
                () -> driver.rightBumper().getAsBoolean(), // Aim at hub or feed point
                () -> V3_Horse_CV2_TRRobotState.getAimAngle().getRadians(),
                () -> 0.0,
                driver.leftTrigger(),
                () -> V3_Horse_CV2_TRRobotState.getCardinalDirection())
            .withName("joystickDriveRotationLock"));

    driver.leftTrigger().onTrue(Commands.runOnce(() -> {}).withName("cardinal-direction-set"));

    driver
        .leftBumper()
        .whileTrue(
            V3_Horse_CV2_TRCompositeCommands.intakeCollect(intake)
                .withName("driver-leftBumper-true"))
        .onFalse(intake.setRollerState(RollerState.IDLE).withName("driver-leftBumper-false"));

    driver
        .rightBumper()
        .whileTrue(
            V3_Horse_CV2_TRCompositeCommands.scoreOrFeedCommand(rollerFloor, shooter)
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
            V3_Horse_CV2_TRCompositeCommands.resetHeading(
                    drive,
                    V3_Horse_CV2_TRRobotState::resetPose,
                    () -> V3_Horse_CV2_TRRobotState.getGlobalPose().getTranslation())
                .withName("driver-dpadDown-true"));

    driver
        .y()
        .whileTrue(
            V3_Horse_CV2_TRCompositeCommands.fixedShotCommand(
                    drive, rollerFloor, shooter, V3_Horse_CV2_TRRobotState.FixedShots.FAR)
                .withName("driver-Y-true"))
        .onFalse(stopShooter("driver-Y-false"));

    driver
        .a()
        .whileTrue(
            V3_Horse_CV2_TRCompositeCommands.fixedShotCommand(
                    drive, rollerFloor, shooter, V3_Horse_CV2_TRRobotState.FixedShots.BUMP)
                .withName("driver-A-true"))
        .onFalse(stopShooter("driver-A-false"));

    driver
        .x()
        .whileTrue(
            V3_Horse_CV2_TRCompositeCommands.fixedShotCommand(
                    drive, rollerFloor, shooter, V3_Horse_CV2_TRRobotState.FixedShots.TRENCH)
                .withName("driver-X-true"))
        .onFalse(stopShooter("driver-X-false"));

    driver
        .b()
        .onTrue(V3_Horse_CV2_TRCompositeCommands.intakeStow(intake).withName("driver-B-true"));

    operator
        .leftBumper()
        .onTrue(
            V3_Horse_CV2_TRCompositeCommands.intakeStow(intake)
                .withName("operator-leftBumper-true"));

    operator
        .rightBumper()
        .whileTrue(
            V3_Horse_CV2_TRCompositeCommands.intakeCollect(intake)
                .withName("operator-rightBumper-true"))
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
                            V3_Horse_CV2_TRRollerFloorConstants.ROLLER_FLOOR_SLOW_VOLTAGE))
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
                            V3_Horse_CV2_TRRollerFloorConstants.ROLLER_FLOOR_RUN_VOLTAGE.times(-1)))
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
    return V3_Horse_CV2_TRCompositeCommands.stopShooter(rollerFloor, shooter).withName(name);
  }

  @Override
  public void robotPeriodic() {
    V3_Horse_CV2_TRRobotState.periodic(
        drive.getRawGyroRotation(), drive.getYawVelocity(), drive.getModulePositions(), drive);
    Logger.recordOutput(
        "Mechanism3D",
        V3_Horse_CV2_TRMechanism3D.getPoses(
            intake.getExtensionPosition(), shooter.getHoodAngle().getMeasure()));
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
