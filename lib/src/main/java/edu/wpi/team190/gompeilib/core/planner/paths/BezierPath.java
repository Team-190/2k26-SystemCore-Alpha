package edu.wpi.team190.gompeilib.core.planner.paths;

import org.wpilib.math.geometry.Pose2d;

import edu.wpi.team190.gompeilib.core.planner.States;

public class BezierPath extends Path {

  public BezierPath(Pose2d startPose, Pose2d endPose, PathConstraints constraints) {
    super(startPose, endPose, constraints);
    //TODO Auto-generated constructor stub
  }

  @Override
  States sample(double time) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'sample'");
  }

  @Override
  States sample(Pose2d pose) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'sample'");
  }

  
}
