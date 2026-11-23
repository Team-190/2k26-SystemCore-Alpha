package edu.wpi.team190.gompeilib.core.planner.paths;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;


public class ArcPath extends Path {

  private final double centerX;
  private final double centerY;
  private final double theta;
  public ArcPath(Pose2d startPose, double theta, double centerX, double centerY, double endRot, PathConstraints constraints) {
    double endX = centerX*Math.cos(theta)+(startPose.getX()-centerX);
    double endY = centerY*Math.sin(theta)+(startPose.getY()-centerY);
    this.centerX = centerX;
    this.centerY = centerY;
    this.theta = theta;
    super(startPose, new Pose2d(new Translation2d(endX,endY), new Rotation2d(endRot)), constraints);
  }

  @Override
  Pose2d sample(double angle) {
    double endX = centerX*Math.cos(angle/theta)+(startPose.getX()-centerX);
    double endY = centerY*Math.sin(angle/theta)+(startPose.getY()-centerY);
    return new Pose2d(new Translation2d(endX,endY), startPose.getRotation().interpolate(endPose.getRotation(), angle/theta));
  }

  @Override
  Pose2d sample(Pose2d pose) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'sample'");
  }
  public void toLog(){
    ArrayList<Pose2d> poseList = new ArrayList<>();
    for(int i = 0; i<=theta; i+=5){
      poseList.add(sample(i));
    }

    Pose2d[] pathList = {};
    Logger.recordOutput("PathListPlanner",poseList.toArray(pathList));
  }
}
