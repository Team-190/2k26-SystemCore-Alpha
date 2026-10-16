package edu.wpi.team190.gompeilib.planner;

import java.util.ArrayList;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class Path {

  private double rate = 10; // talk abt ts later
  private ArrayList<State> states = new ArrayList<State>();

  public Path(Pose2d startPose, Pose2d endPose, double time) {
    double deltaX = endPose.getX() - startPose.getX();
    double deltaY = endPose.getY() - startPose.getY();
    double angVel = endPose.getRotation().minus(startPose.getRotation()).getRadians() / time;
    ChassisVelocities vel = new ChassisVelocities(deltaX / time, deltaY / time, angVel);

    int stamps = (int) Math.ceil(time * rate);
    for (int i = 0; i <= stamps; i++) {
      double t = (double) i / stamps;
      Pose2d pose =
          new Pose2d(
              startPose.getTranslation().interpolate(endPose.getTranslation(), t),
              startPose.getRotation().interpolate(endPose.getRotation(), t));
      states.add(new State(pose, vel));
    }
  }

  public ArrayList<State> getStates() {
    return states;
  }
}

enum PathType {
  LINEAR,
  BEZIER
}

// this is how the different paths are defined
