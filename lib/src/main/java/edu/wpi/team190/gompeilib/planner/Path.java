package edu.wpi.team190.gompeilib.planner;

import java.util.ArrayList;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;

public class Path {

  public static final double RATE = 10; // states per second
  private ArrayList<State> states = new ArrayList<State>();
  private double distance;

  public Path(Pose2d startPose, Pose2d endPose, double time) {
    distance = startPose.getTranslation().getDistance(endPose.getTranslation());
    // makes a state every 1 / RATE seconds from start to end (including the end) basically just
    // creates small states along the entire thing
    // at least 1 step, otherwise a time of 0 divides by zero and fills the path with NaN
    int numSteps = Math.max(1, (int) Math.ceil(time * RATE));
    // time gets rounded up to fit the steps baisically (0.73s -> 0.8s) so the velocity matches the
    // states
    double actualTime = numSteps / RATE;

    double deltaX = endPose.getX() - startPose.getX();
    double deltaY = endPose.getY() - startPose.getY();
    double deltaAngle = endPose.getRotation().minus(startPose.getRotation()).getRadians();

    double velocityX = deltaX / actualTime;
    double velocityY = deltaY / actualTime;
    double turnSpeed = deltaAngle / actualTime;
    ChassisVelocities velocity = new ChassisVelocities(velocityX, velocityY, turnSpeed);

    for (int step = 0; step <= numSteps; step++) {
      double fraction = (double) step / numSteps; // 0 at start, 1 at end
      Pose2d pose =
          new Pose2d(
              startPose.getTranslation().interpolate(endPose.getTranslation(), fraction),
              startPose.getRotation().interpolate(endPose.getRotation(), fraction));
      states.add(new State(pose, velocity, fraction));
    }
  }

  public ArrayList<State> getStates() {
    return states;
  }

  public double getDistance(){
    return distance;
  }
}

// this is how the different paths are defined
