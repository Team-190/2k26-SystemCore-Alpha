package edu.wpi.team190.gompeilib.core.planner.paths;

import org.wpilib.math.geometry.Pose2d;

public class RotationalPath extends Path {

  public RotationalPath(Pose2d startPose, Pose2d endPose, PathConstraints constraints) {
    super(startPose, endPose, constraints);
    // TODO Auto-generated constructor stub
  }

  @Override
  Pose2d sample(double time) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'sample'");
  }

  @Override
  Pose2d sample(Pose2d pose) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'sample'");
  }
}
