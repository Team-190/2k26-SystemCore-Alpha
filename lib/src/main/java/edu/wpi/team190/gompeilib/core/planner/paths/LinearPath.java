package edu.wpi.team190.gompeilib.core.planner.paths;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;

public class LinearPath extends Path {
  private final double time;
  private double pastTime = 0.02;
  private Supplier<ChassisVelocities> vel;

  public LinearPath(Pose2d startPose, Pose2d endPose, PathConstraints constraints, Supplier<ChassisVelocities> vel) {
    super(startPose, endPose, constraints);
    time = calculateTime();
    this.vel = vel;
  }

  @Override
  public Pose2d sample(double t) {
    return startPose.interpolate(endPose, t);
  }


  @Override
  public Pose2d sample(Pose2d pose) {
    Pose2d newPose = pose.interpolate(pose, vel.get());
    Logger.recordOutput("TIMELINEARPATH", newPose);
    return newPose;
  }

  public void LogPath() {
    ArrayList<Pose2d> poseList = new ArrayList<Pose2d>();
    poseList.add(startPose);
    for (int i = 0; i <= 10; i++) {
      poseList.add(sample(poseList.get(i)));
    }
    Pose2d[] poseArray = {};
    Logger.recordOutput("LINEARPATHLISSI", poseList.toArray(poseArray));
  }

  private double calculateTime() {
    double totalTime;
    double accTime = constraints.maxVelocity / constraints.maxAcceleration;
    double accDist = (constraints.maxVelocity / 2) * accTime;
    double totalDist = (startPose.getTranslation().getDistance(endPose.getTranslation()));
    double constDist = totalDist - accDist;
    if (constDist < 0) {
      double vf = Math.sqrt(2 * constraints.maxAcceleration * totalDist);
      totalTime = vf / constraints.maxAcceleration;
    } else {
      double constTime = constDist / constraints.maxVelocity;
      totalTime = accTime + constTime;
    }
    return totalTime;
  }
}
