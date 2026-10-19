// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.v3_Horse_CV2_TR.subsystems;

import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIO;
import edu.wpi.team190.gompeilib.planner.Path;
import edu.wpi.team190.gompeilib.planner.Plan;
import edu.wpi.team190.gompeilib.planner.Planner;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOSim;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

public class V3_Horse_CV2_TRRobotContainer {

  private SwerveDrive drive;
  private Plan autoPlan;
  private Plan cornerTestPlan;

  public V3_Horse_CV2_TRRobotContainer() {

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
            (Pose2d pose) -> {});
    Planner.setDriveCommand(drive::runVelocity);
    Pose2d start = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    Pose2d endOfStraight = new Pose2d(4, 0, Rotation2d.fromDegrees(0));
    Pose2d gentleTurn = new Pose2d(7, 1, Rotation2d.fromDegrees(0));
    Pose2d rotatedWhileDriving = new Pose2d(10, 1, Rotation2d.fromDegrees(90));
    Pose2d mediumTurn = new Pose2d(12, 3.5, Rotation2d.fromDegrees(90));
    Pose2d turnedInPlace = new Pose2d(12, 3.5, Rotation2d.fromDegrees(-90));
    Pose2d sharpTurn = new Pose2d(8, 5.5, Rotation2d.fromDegrees(-90));
    Pose2d shortSegment = new Pose2d(7.5, 5.5, Rotation2d.fromDegrees(-90));
    Pose2d endOfSlowZone = new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(180));
    Pose2d end = new Pose2d(0, 3, Rotation2d.fromDegrees(180));
    autoPlan =
        new Plan(
            new Path(start, endOfStraight, 4, 3),
            new Path(endOfStraight, gentleTurn, 4, 3),
            new Path(gentleTurn, rotatedWhileDriving, 4, 3),
            new Path(rotatedWhileDriving, mediumTurn, 4, 3),
            new Path(mediumTurn, turnedInPlace, 4, 3),
            new Path(turnedInPlace, sharpTurn, 4, 3),
            new Path(sharpTurn, shortSegment, 4, 3),
            new Path(shortSegment, endOfSlowZone, 1.5, 3),
            new Path(endOfSlowZone, end, 4, 3));

    Pose2d squareStart = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    Pose2d sharpCorner = new Pose2d(4, 0, Rotation2d.fromDegrees(0));
    Pose2d smallCurve = new Pose2d(4, 4, Rotation2d.fromDegrees(0));
    Pose2d bigCurve = new Pose2d(0, 4, Rotation2d.fromDegrees(0));
    cornerTestPlan =
        new Plan(
            new Path(squareStart, sharpCorner, 3, 3),
            new Path(sharpCorner, smallCurve, 3, 3, 0.5),
            new Path(smallCurve, bigCurve, 3, 3, 1.0),
            new Path(bigCurve, squareStart, 3, 3));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Planner.followPath(cornerTestPlan, V3_Horse_CV2_TRRobotState::getGlobalPose);
  }

  public void robotPeriodic() {
    V3_Horse_CV2_TRRobotState.addOdometryObservation(
        drive.getRawGyroRotation(), drive.getModulePositions());
  }

  public Pose2d getPose() {
    return V3_Horse_CV2_TRRobotState.getGlobalPose();
  }
}
