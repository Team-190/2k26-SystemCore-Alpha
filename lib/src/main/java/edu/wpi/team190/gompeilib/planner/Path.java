package edu.wpi.team190.gompeilib.planner;

import edu.wpi.team190.gompeilib.planner.Containers.PosePlus;
import java.util.ArrayList;
import org.wpilib.math.geometry.Pose2d;

public class Path {

  private double rate = 10; // talk abt ts later
  private ArrayList<PosePlus> posePath = new ArrayList<PosePlus>();

  public Path(Pose2d startPose, Pose2d endPose, double time) {
    double desiredVel = startPose.getTranslation().getDistance(endPose.getTranslation()) / time;
    double angVel = endPose.getRotation().getRadians() - startPose.getRotation().getRadians();
    double stamps = time * rate;
    for (int i = 0; i < stamps; i++) {
      posePath.add(new PosePlus(startPose.interpolate(endPose, 1 / stamps), desiredVel, angVel));
    }
  }

  public ArrayList<PosePlus> getPosePath() {
    return posePath;
  }
}

enum PathType {
  LINEAR,
  BEZIER
}

// this is how the different paths are defined
