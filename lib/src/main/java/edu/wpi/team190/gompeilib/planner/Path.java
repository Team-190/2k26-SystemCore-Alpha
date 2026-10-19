package edu.wpi.team190.gompeilib.planner;

import org.wpilib.math.geometry.Pose2d;

public class Path {

  public static final double RATE = 10; // states per second
  private final Pose2d startPose;
  private final Pose2d endPose;
  private final double maxVel;
  private final double maxAccel;
  private final double endRadius;

  public Path(Pose2d startPose, Pose2d endPose, double maxVel, double maxAccel) {
    this(startPose, endPose, maxVel, maxAccel, 0);
  }

  public Path(Pose2d startPose, Pose2d endPose, double maxVel, double maxAccel, double endRadius) {
    this.startPose = startPose;
    this.endPose = endPose;
    this.maxAccel = maxAccel;
    this.maxVel = maxVel;
    this.endRadius = endRadius;
  }

  public Pose2d getStartPose() {
    return startPose;
  }

  public Pose2d getEndPose() {
    return endPose;
  }

  public double getMaxVel() {
    return maxVel;
  }

  public double getMaxAccel() {
    return maxAccel;
  }

  public double getEndRadius() {
    return endRadius;
  }
}

// this is how the different paths are defined
