// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.v3_Horse_CV2_TR.subsystems;

import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIO;
import edu.wpi.team190.gompeilib.planner.PathNode;
import edu.wpi.team190.gompeilib.planner.Planner;
import edu.wpi.team190.gompeilib.planner.PlannerControllerConfig;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOSim;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.command2.Command;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.trajectory.TrapezoidProfile;

public class V3_Horse_CV2_TRRobotContainer {

  private SwerveDrive drive;
  private ArrayList<PathNode> path =
      new ArrayList<PathNode>(
          List.of(new PathNode(new Pose2d(new Translation2d(3, 3), new Rotation2d(45)), 2)));

  public V3_Horse_CV2_TRRobotContainer() {

    PlannerControllerConfig plannerConfig =
        new PlannerControllerConfig(
            new PIDController(1, 1, 1),
            new PIDController(1, 1, 1),
            new ProfiledPIDController(1, 1, 1, new TrapezoidProfile.Constraints(1, 1)));

    Planner.setConfig(plannerConfig);
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
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Planner.followPath(path, V3_Horse_CV2_TRRobotState::getGlobalPose);
  }

  public void robotPeriodic() {
    V3_Horse_CV2_TRRobotState.addOdometryObservation(
        drive.getRawGyroRotation(), drive.getModulePositions());
  }

  public Pose2d getPose() {
    return V3_Horse_CV2_TRRobotState.getGlobalPose();
  }
}
