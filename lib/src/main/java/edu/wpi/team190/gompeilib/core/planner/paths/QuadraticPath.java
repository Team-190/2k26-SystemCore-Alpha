package edu.wpi.team190.gompeilib.core.planner.paths;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;


public class QuadraticPath extends Path {
  private final Translation2d start;
  private final Translation2d control;
  private final Translation2d end;

  public QuadraticPath(Translation2d start, Translation2d control, Translation2d end) {
    this.start = start;
    this.control = control;
    this.end = end;
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
