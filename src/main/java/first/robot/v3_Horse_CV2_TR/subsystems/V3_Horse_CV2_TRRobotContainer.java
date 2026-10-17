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
import org.wpilib.math.geometry.Translation2d;

public class V3_Horse_CV2_TRRobotContainer {

  private SwerveDrive drive;
  private Plan autoPlan;

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
    autoPlan =
        new Plan(
            new Path(
                new Pose2d(new Translation2d(0, 0), new Rotation2d()),
                new Pose2d(new Translation2d(6, 7), new Rotation2d()),
                4,
                1));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Planner.followPath(autoPlan, V3_Horse_CV2_TRRobotState::getGlobalPose);
  }

  public void robotPeriodic() {
    V3_Horse_CV2_TRRobotState.addOdometryObservation(
        drive.getRawGyroRotation(), drive.getModulePositions());
  }

  public Pose2d getPose() {
    return V3_Horse_CV2_TRRobotState.getGlobalPose();
  }
}
