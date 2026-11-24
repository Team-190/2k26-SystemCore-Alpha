package edu.wpi.team190.gompeilib.core.planner.paths;

import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class RotationalPath extends Path {

  private final Rotation2d rotation;

  public RotationalPath(Pose2d startPose, Rotation2d rotation, Rotation2d angVel) {
    this.rotation = rotation;
    super(
        startPose,
        new Pose2d(startPose.getTranslation(), rotation),
        new PathConstraints(0, 0, angVel.getRadians()));
  }

  @Override
  Pose2d sample(double time) {
    return new Pose2d(
        startPose.getTranslation(), startPose.getRotation().interpolate(rotation, time));
  }

  @Override
  Pose2d sample(Pose2d pose, ChassisVelocities vel) {
    return new Pose2d(
        startPose.getTranslation(), pose.getRotation().plus(new Rotation2d(vel.omega)));
  }

  public void toLog() {
    ArrayList<Rotation2d> rotlist = new ArrayList<>();
    for (double i = 0; i <= 1; i += 0.1) {
      rotlist.add(sample(i).getRotation());
    }
    Rotation2d[] rotList = {};
    rotlist.toArray(rotList);
    Logger.recordOutput("ROTOTOT", rotList);
  }
}
