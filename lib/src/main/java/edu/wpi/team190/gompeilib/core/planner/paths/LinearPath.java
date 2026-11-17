package edu.wpi.team190.gompeilib.core.planner.paths;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

import edu.wpi.team190.gompeilib.core.planner.States;

public class LinearPath extends Path {
  private final Translation2d start;
  private final Translation2d end;

  public LinearPath(Translation2d start, Translation2d end) {
    this.start = start;
    this.end = end;
  }

  @Override
  public States sample(double t) {
     throw new UnsupportedOperationException("Unimplemented method 'sample'");
  }

  @Override
  States sample(Pose2d pose) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'sample'");
  }
}
