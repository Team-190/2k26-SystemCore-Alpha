package edu.wpi.team190.gompeilib.core.planner.paths;

import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class ArcPath extends Path {

  private final double centerX;
  private final double centerY;
  private final double theta;
  private ChassisVelocities vel;

  public ArcPath(
      Pose2d startPose,
      double theta,
      double centerX,
      double centerY,
      double endRot,
      PathConstraints constraints) {
    double endX = centerX * Math.cos(theta) + (startPose.getX() - centerX);
    double endY = centerY * Math.sin(theta) + (startPose.getY() - centerY);
    this.centerX = centerX;
    this.centerY = centerY;
    this.theta = theta;
    super(
        startPose, new Pose2d(new Translation2d(endX, endY), new Rotation2d(endRot)), constraints);
  }

  @Override
  Pose2d sample(double t) {
    double endX = centerX * Math.cos(Math.toRadians(t * theta)) + (startPose.getX() - centerX);
    double endY = centerY * Math.sin(Math.toRadians(t * theta)) + (startPose.getY() - centerY);
    return new Pose2d(
        new Translation2d(endX, endY),
        startPose.getRotation().interpolate(endPose.getRotation(), t));
  }

  public void setVel(ChassisVelocities vel) {
    this.vel = vel;
  }

  @Override
  Pose2d sample(Pose2d pose) {
    return new Pose2d(
        startPose
            .getTranslation()
            .plus(
                new Twist2d(vel.vx, vel.vy, pose.getRotation().getRadians())
                    .exp()
                    .getTranslation()),
        startPose.getRotation().plus(new Rotation2d(vel.omega)));
  }

  public void toLog() {
    ArrayList<Pose2d> poseList = new ArrayList<>();
    for (double i = 0; i < 1; i += 0.1) {
      System.out.print(i);
      poseList.add(sample(i));
    }
    Pose2d[] pathList = {};
    Logger.recordOutput("PathListPlanner", poseList.toArray(pathList));
  }
}
