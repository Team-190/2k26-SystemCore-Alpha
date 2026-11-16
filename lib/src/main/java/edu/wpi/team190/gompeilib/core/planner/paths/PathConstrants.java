package edu.wpi.team190.gompeilib.core.planner.paths;

public class PathConstrants {

  public double maxAcceleration;
  public double maxVelocity;
  public double maxThetaVelocity;
  public double timeInital;
  public double timeFinal;

  public PathConstrants(double maxAcceleration, double maxVelocity, double maxThetaVelocity) {
    this.maxAcceleration = maxAcceleration;
    this.maxVelocity = maxVelocity;
    this.maxThetaVelocity = maxThetaVelocity;
  }

  public void setZone(double timeInital, double timeFinal) {
    this.timeInital = timeInital;
    this.timeFinal = timeFinal;
  }
}
