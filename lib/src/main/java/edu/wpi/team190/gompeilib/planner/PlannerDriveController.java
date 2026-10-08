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
  private int count = 0;

  private final PIDController m_xController;
  private final PIDController m_yController;
  private final ProfiledPIDController m_thetaController;

  public PlannerDriveController(PlannerControllerConfig config) {
    m_xController = config.m_xController();
    m_yController = config.m_yController();
    m_thetaController = config.m_thetaController();
    m_thetaController.enableContinuousInput(0, Math.PI * 2);
  }

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
      double desiredLinearVelocity // m/s velocity at time of call
      ) {
    if (m_firstRun) {
      m_thetaController.reset(currentPose.getRotation().getRadians());
      m_firstRun = false;
    }
    Rotation2d rotation =
        currentPose.getTranslation().minus(trajectoryPose.getTranslation()).getAngle();
    // calculate feedforward velocities
    double xFF =
        -1 * (currentPose.getTranslation().getX() - trajectoryPose.getTranslation().getX());
    double yFF =
        -1 * (currentPose.getTranslation().getY() - trajectoryPose.getTranslation().getY());
    // ts is the issue rn
    Logger.recordOutput("rotation", rotation);
    Logger.recordOutput("xFF", xFF);
    Logger.recordOutput("yFF", yFF);
    double thetaFF = 0;

    ChassisVelocities vel =
        new ChassisVelocities(xFF, yFF, thetaFF).toRobotRelative(currentPose.getRotation());
    Logger.recordOutput("currentPose", currentPose);
    Logger.recordOutput("trajectoryPose", trajectoryPose);
    Logger.recordOutput("PlannerVel", vel);
    Logger.recordOutput("count", count);
    count++;
    return vel;
  }
}
