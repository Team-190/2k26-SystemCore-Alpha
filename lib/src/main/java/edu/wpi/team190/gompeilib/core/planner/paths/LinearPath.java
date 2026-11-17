package edu.wpi.team190.gompeilib.core.planner.paths;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

public class LinearPath extends Path {
  private final double time;

  public LinearPath(Pose2d startPose, Pose2d endPose, PathConstraints constraints) {
    super(startPose,endPose,constraints);
    time = calculateTime();
    
  }

  @Override
  public Pose2d sample(double t) {
    return startPose.interpolate(endPose, t/time);
  }

  @Override
  Pose2d sample(Pose2d pose) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'sample'");
  }

 public double calculateTime(){
    double totalTime;
    double accTime = constraints.maxVelocity/constraints.maxAcceleration;
    double accDist = (constraints.maxVelocity/2)*accTime;
    double totalDist = (startPose.getTranslation().getDistance(endPose.getTranslation()));
    double constDist = totalDist-accDist;
    if(constDist < 0){
        double vf = Math.sqrt(2*constraints.maxAcceleration*totalDist);
        totalTime = vf/constraints.maxAcceleration;
    }else{
        double constTime = constDist/constraints.maxVelocity;
        totalTime = accTime+constTime;
    }
    return totalTime;
 }
}
