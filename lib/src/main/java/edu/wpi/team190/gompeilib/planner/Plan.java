package edu.wpi.team190.gompeilib.planner;

import edu.wpi.team190.gompeilib.planner.Containers.PosePlus;
import edu.wpi.team190.gompeilib.planner.Containers.State;
import java.util.ArrayList;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class Plan {

  private final ArrayList<PosePlus> combinedPath = new ArrayList<PosePlus>();

  public Plan(ArrayList<PosePlus>... paths) {

    for (ArrayList<PosePlus> path : paths) {
      combinedPath.addAll(path);
    }
  }

  public ArrayList<State> getPlan() {

    ArrayList<State> statePath = new ArrayList<State>();

    for (int i = 0; i < combinedPath.size() - 1; i++) {
      Pose2d currentPose = combinedPath.get(i).pose();
      Pose2d nextPose = combinedPath.get(i + 1).pose();
      double desiredVel = combinedPath.get(i + 1).vel();
      double angVel = combinedPath.get(i + 1).angVel();

      double deltaX = nextPose.getTranslation().getX() - currentPose.getTranslation().getX();
      double deltaY = nextPose.getTranslation().getY() - currentPose.getTranslation().getY();
      double theta = Math.atan2(deltaY, deltaX);
      double velX = desiredVel * Math.cos(theta);
      double velY = desiredVel * Math.sin(theta);

      ChassisVelocities vel = new ChassisVelocities(velX, velY, angVel);
      statePath.add(new State(currentPose, vel));
    }
    return statePath;
  }
}
// puts together the paths and generates states along the path
