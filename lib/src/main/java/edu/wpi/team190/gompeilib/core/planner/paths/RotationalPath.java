package edu.wpi.team190.gompeilib.core.planner.paths;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class RotationalPath extends Path {

  private final Rotation2d rotation;

  public RotationalPath(Pose2d startPose, Rotation2d rotation, Rotation2d angVel) {
    this.rotation = rotation;
    super(
        startPose,
        new Pose2d(startPose.getTranslation(), rotation),
        new PathConstraints(0, 0, angVel.getRadians()));
  }

  @Override
  Pose2d sample(double time) {
    Pose2d pose =
        new Pose2d(startPose.getTranslation(), startPose.getRotation().interpolate(rotation, time));
    return pose;
  }

  @Override
  Pose2d sample(Pose2d pose, ChassisVelocities vel) {
    return new Pose2d(
        startPose.getTranslation(), pose.getRotation().plus(new Rotation2d(vel.omega)));
  }
}
