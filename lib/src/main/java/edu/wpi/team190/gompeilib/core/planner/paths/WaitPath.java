package edu.wpi.team190.gompeilib.core.planner.paths;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;


public class WaitPath extends Path {

  private final Rotation2d rotation;
  public WaitPath(Pose2d startPose, Rotation2d rotation, Rotation2d angVel) {
    this.rotation = rotation;
    super(startPose, new Pose2d(startPose.getTranslation(), rotation), new PathConstraints(0,0,angVel.getRadians()));
  }

  @Override
  Pose2d sample(double time) {
    return new Pose2d(startPose.getTranslation(),startPose.getRotation().interpolate(rotation, time));
  }

  @Override
  Pose2d sample(Pose2d pose, ChassisVelocities vel) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'sample'");
  }
}
