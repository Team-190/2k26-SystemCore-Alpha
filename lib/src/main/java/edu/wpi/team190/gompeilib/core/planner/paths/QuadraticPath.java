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
  public Pose2d sample(double t) {
    double u = 1.0 - t;

    return new Pose2d(
        new Translation2d(
            u * u * start.getX() + 2 * u * t * control.getX() + t * t * end.getX(),
            u * u * start.getY() + 2 * u * t * control.getY() + t * t * end.getY()),
        new Rotation2d());
  }

  @Override
  Pose2d sample(Pose2d pose) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'sample'");
  }
}
