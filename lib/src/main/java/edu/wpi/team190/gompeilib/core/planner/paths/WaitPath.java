package edu.wpi.team190.gompeilib.core.planner.paths;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class WaitPath extends Path {

  public WaitPath(Pose2d startPose, Pose2d endPose, PathConstraints constraints) {
    super(startPose, endPose, constraints);
    // TODO Auto-generated constructor stub
  }

  @Override
  Pose2d sample(double time) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'sample'");
  }

  @Override
  Pose2d sample(Pose2d pose, ChassisVelocities vel) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'sample'");
  }
}
