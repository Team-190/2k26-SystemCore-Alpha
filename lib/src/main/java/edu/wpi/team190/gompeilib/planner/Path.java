package edu.wpi.team190.gompeilib.planner;

import java.util.ArrayList;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class Path {

  public static final double RATE = 10; // states per second
  private double distance;
  private Pose2d startPose;
  private Pose2d endPose;
  private double maxVel;
  private double maxAccel;
  public Path(Pose2d startPose, Pose2d endPose, double maxVel, double maxAccel) {
    this.startPose = startPose;
    this.endPose = endPose;
    this.maxAccel = maxAccel;
    this.maxVel = maxVel;
  }

  public Pose2d getStartPose() {
    return startPose;
  }
  
  public Pose2d getEndPose() {
    return endPose;
  }

  public double getMaxVel(){
    return maxVel;
  }
  
  public double getMaxAccel(){
    return maxAccel;
  }
}

// this is how the different paths are defined
