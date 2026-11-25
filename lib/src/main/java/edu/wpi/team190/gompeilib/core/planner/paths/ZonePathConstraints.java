package edu.wpi.team190.gompeilib.core.planner.paths;

/** ZonePathConstraints */
public class ZonePathConstraints extends PathConstraints {

  private final double timeInital;
  private final double timeFinal;

  public ZonePathConstraints(
      double maxAcceleration,
      double maxVelocity,
      double maxThetaVelocity,
      double timeInital,
      double timeFinal) {
    this.timeInital = timeInital;
    this.timeFinal = timeFinal;
    super(maxAcceleration, maxVelocity, maxThetaVelocity);
  }
}
