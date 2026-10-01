package edu.wpi.team190.gompeilib.planner;

import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

public class Segment {
  public final Optional<Rotation2d> rotation;
  public final Optional<Translation2d> translation;
  public boolean constainToLine = false;

  public Segment(Pose2d pose) {
    rotation = Optional.of(pose.getRotation());
    translation = Optional.of(pose.getTranslation());
  }

  public Segment(Rotation2d rotation2d) {
    rotation = Optional.of(rotation2d);
    translation = Optional.ofNullable(null);
  }

  public Segment(Translation2d translation2d) {
    rotation = Optional.ofNullable(null);
    translation = Optional.of(translation2d);
  }

  public Segment constrainToLine() {
    constainToLine = true;
    return this;
  }
  // public Segment desiredVelo
}
