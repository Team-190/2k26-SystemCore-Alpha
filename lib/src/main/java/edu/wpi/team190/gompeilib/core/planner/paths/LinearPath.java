package edu.wpi.team190.gompeilib.core.planner.paths;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

public class LinearPath extends Path {
  private final Translation2d start;
  private final Translation2d end;

  public LinearPath(Translation2d start, Translation2d end) {
    this.start = start;
    this.end = end;
  }

  @Override
  public Pose2d sample(double t) {
    return new Pose2d(
        new Translation2d(
            start.getX() + (end.getX() - start.getX()) * t,
            start.getY() + (end.getY() - start.getY()) * t),
        new Rotation2d());
  }

  @Override
  Pose2d sample(Pose2d pose) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'sample'");
  }
}
