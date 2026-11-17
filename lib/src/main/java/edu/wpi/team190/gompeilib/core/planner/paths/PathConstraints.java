package edu.wpi.team190.gompeilib.core.planner.paths;

public class PathConstraints {

  public final double maxAcceleration;
  public final double maxVelocity;
  public final double maxThetaVelocity;

  //for zones
  public double timeInital;
  public double timeFinal;

  public PathConstraints(double maxAcceleration, double maxVelocity, double maxThetaVelocity) {
    this.maxAcceleration = maxAcceleration;
    this.maxVelocity = maxVelocity;
    this.maxThetaVelocity = maxThetaVelocity;
  }

  public void setZone(double timeInital, double timeFinal) {
    this.timeInital = timeInital;
    this.timeFinal = timeFinal;
  }
}
