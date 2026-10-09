package edu.wpi.team190.gompeilib.planner;

import org.littletonrobotics.junction.Logger;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class PlannerDriveController {
  private Pose2d m_poseError = new Pose2d();
  private Rotation2d m_rotationError = new Rotation2d();
  private double m_poseTolerance = 1;
  private Boolean m_firstRun = true;

  public boolean atReference(Pose2d currentPose, Pose2d trajectoryPose) {
    return (m_poseTolerance
        >= currentPose.getTranslation().getDistance(trajectoryPose.getTranslation()));
  }

  public void setTolerance(double tolerance) {
    m_poseTolerance = tolerance;
  }

  public ChassisVelocities calculate(
      Pose2d currentPose,
      Pose2d trajectoryPose,
      double desiredLinearVelocity 
      ) {
    double xFF =
        -1 * (currentPose.getTranslation().getX() - trajectoryPose.getTranslation().getX());
    double yFF =
        -1 * (currentPose.getTranslation().getY() - trajectoryPose.getTranslation().getY());
    Logger.recordOutput("xFF", xFF);
    Logger.recordOutput("yFF", yFF);
    double thetaFF =
        (currentPose.getRotation().getDegrees() - trajectoryPose.getRotation().getDegrees())
            * -0.07;

    ChassisVelocities vel =
        new ChassisVelocities(xFF, yFF, thetaFF).toRobotRelative(currentPose.getRotation());
    Logger.recordOutput("currentPose", currentPose);
    Logger.recordOutput("trajectoryPose", trajectoryPose);
    Logger.recordOutput("PlannerVel", vel);
    return vel;
  }
}
