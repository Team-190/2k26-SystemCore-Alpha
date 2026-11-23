package edu.wpi.team190.gompeilib.core.planner.paths;

import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class LinearPath extends Path {
  private Supplier<ChassisVelocities> vel;

  public LinearPath(
      Pose2d startPose,
      Pose2d endPose,
      PathConstraints constraints,
      Supplier<ChassisVelocities> vel) {
    super(startPose, endPose, constraints);
    this.vel = vel;
  }

  @Override
  public Pose2d sample(double t) {
    return new Pose2d(
        startPose.getTranslation().interpolate(endPose.getTranslation(), t),
        startPose.getRotation().interpolate(endPose.getRotation(), t));
  }

  @Override
  public Pose2d sample(Pose2d pose) {
    double vy = vel.get().vy;
    double vx = vel.get().vx;
    double dist =
        (Math.sqrt(vy * vy + vx * vx) * 0.02)
            / startPose.getTranslation().getDistance(endPose.getTranslation());
    Pose2d newPose =
        new Pose2d(
            pose.getTranslation().interpolate(endPose.getTranslation(), dist),
            pose.getRotation().interpolate(endPose.getRotation(), dist));
    return newPose;
  }
}
