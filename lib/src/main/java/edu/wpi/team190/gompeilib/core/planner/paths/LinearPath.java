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
    return startPose.interpolate(endPose, t);
  }

  @Override
  public Pose2d sample(Pose2d pose) {
    double vy = vel.get().vy;
    double vx = vel.get().vx;
    Pose2d newPose =
        pose.interpolate(
            pose,
            (Math.sqrt(vy * vy + vx * vx) * 0.02)
                / startPose.getTranslation().getDistance(endPose.getTranslation()));
    return newPose;
  }

  public void LogPath() {
    ArrayList<Pose2d> poseList = new ArrayList<Pose2d>();
    poseList.add(startPose);
    for (int i = 0; i <= 10; i++) {
      poseList.add(sample(i / 10.0));
    }
    Pose2d[] poseArray = {};
    Logger.recordOutput("LINEARPATHLISSI", poseList.toArray(poseArray));
  }
}
