package edu.wpi.team190.gompeilib.core.planner.paths;

public class PathConstraints {

  private final double maxAcceleration;
  private final double maxVelocity;
  private final double maxThetaVelocity;

  public PathConstraints(double maxAcceleration, double maxVelocity, double maxThetaVelocity) {
    this.maxAcceleration = maxAcceleration;
    this.maxVelocity = maxVelocity;
    this.maxThetaVelocity = maxThetaVelocity;
  }

  public double getMaxAccel() {
    return maxAcceleration;
  }

  public double getMaxVel() {
    return maxVelocity;
  }

  public double getMaxRotVel() {
    return maxThetaVelocity;
  }
}
