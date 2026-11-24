package edu.wpi.team190.gompeilib.core.planner.paths;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class ArcPath extends Path {

  private final double centerX;
  private final double centerY;
  private final double theta;

  public ArcPath(
      Pose2d startPose,
      Rotation2d rotation,
      Translation2d translation,
      Rotation2d endrot,
      PathConstraints constraints) {
    double centerX = translation.getX();
    double centerY = translation.getY();
    double theta = rotation.getRadians();
    double endX = centerX * Math.cos(theta) + (startPose.getX() - centerX);
    double endY = centerY * Math.sin(theta) + (startPose.getY() - centerY);
    this.centerX = centerX;
    this.centerY = centerY;
    this.theta = theta;
    super(startPose, new Pose2d(new Translation2d(endX, endY), endrot), constraints);
  }

  @Override
  Pose2d sample(double t) {
    double endX = centerX * Math.cos(Math.toRadians(t * theta)) + (startPose.getX() - centerX);
    double endY = centerY * Math.sin(Math.toRadians(t * theta)) + (startPose.getY() - centerY);
    return new Pose2d(
        new Translation2d(endX, endY),
        startPose.getRotation().interpolate(endPose.getRotation(), t));
  }

  @Override
  Pose2d sample(Pose2d pose, ChassisVelocities vel) {
    return new Pose2d(
        startPose
            .getTranslation()
            .plus(
                new Twist2d(vel.vx, vel.vy, pose.getRotation().getRadians())
                    .exp()
                    .getTranslation()),
        startPose.getRotation().plus(new Rotation2d(vel.omega)));
  }

  @Override
  void calculateTime() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'calculateTime'");
  }
}
