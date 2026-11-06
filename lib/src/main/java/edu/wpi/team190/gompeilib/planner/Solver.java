package edu.wpi.team190.gompeilib.planner;

// solves ACTUAL chassis velocities based on current state and goal state
public class Solver {
  // both gains are per second: a 1 m / 1 rad error turns into 1 m/s / 1 rad/s of correction
  private static final double DEFAULT_TRANSLATION_GAIN = 1.0;
  private static final double DEFAULT_ROTATION_GAIN = 4.0;
  private static final double DEFAULT_TRANSLATION_TOLERANCE = 0.05; // meters
  private static final double DEFAULT_ROTATION_TOLERANCE = Math.toRadians(2);
  private static final double DEFAULT_MAX_LINEAR_VELOCITY = 4.5;
  private static final double DEFAULT_MAX_ANGULAR_VELOCITY = 2 * Math.PI;

  private final double translationGain;
  private final double rotationGain;
  private final double translationTolerance;
  private final double rotationTolerance;
  private final double maxLinearVelocity;
  private final double maxAngularVelocity;

  public Solver() {
    this(
        DEFAULT_TRANSLATION_GAIN,
        DEFAULT_ROTATION_GAIN,
        DEFAULT_TRANSLATION_TOLERANCE,
        DEFAULT_ROTATION_TOLERANCE,
        DEFAULT_MAX_LINEAR_VELOCITY,
        DEFAULT_MAX_ANGULAR_VELOCITY);
  }

  public Solver(
      double translationGain,
      double rotationGain,
      double translationTolerance,
      double rotationTolerance) {
    this(
        translationGain,
        rotationGain,
        translationTolerance,
        rotationTolerance,
        DEFAULT_MAX_LINEAR_VELOCITY,
        DEFAULT_MAX_ANGULAR_VELOCITY);
  }

  public Solver(
      double translationGain,
      double rotationGain,
      double translationTolerance,
      double rotationTolerance,
      double maxLinearVelocity,
      double maxAngularVelocity) {
    this.translationGain = translationGain;
    this.rotationGain = rotationGain;
    this.translationTolerance = translationTolerance;
    this.rotationTolerance = rotationTolerance;
    this.maxLinearVelocity = maxLinearVelocity;
    this.maxAngularVelocity = maxAngularVelocity;
  }

  public static Solver forDrivetrain(double maxLinearVelocity, double maxAngularVelocity) {
    return new Solver(
        DEFAULT_TRANSLATION_GAIN,
        DEFAULT_ROTATION_GAIN,
        DEFAULT_TRANSLATION_TOLERANCE,
        DEFAULT_ROTATION_TOLERANCE,
        maxLinearVelocity,
        maxAngularVelocity);
  }

  // returns FIELD relative {vx, vy, omega}, the robot turns it into robot relative with its own
  // gyro right before driving so it isnt using an old heading if this ran somewhere else
  public double[] calculate(
      double robotX,
      double robotY,
      double robotHeading,
      double goalX,
      double goalY,
      double goalHeading,
      double plannedVelocityX,
      double plannedVelocityY,
      double plannedTurnSpeed) {
    // bad plan, nothing safe to follow so stop
    if (!isFinite(plannedVelocityX, plannedVelocityY, plannedTurnSpeed)) {
      return new double[] {0, 0, 0};
    }
    if (!isFinite(robotX, robotY, robotHeading)) {
      return new double[] {0, 0, 0};
    }
    if (!isFinite(goalX, goalY, goalHeading)) {
      return new double[] {plannedVelocityX, plannedVelocityY, plannedTurnSpeed};
    }

    double errorX = goalX - robotX;
    double errorY = goalY - robotY;
    double headingError = headingError(robotHeading, goalHeading);

    double correctionX = translationGain * errorX;
    double correctionY = translationGain * errorY;
    double turnCorrection = rotationGain * headingError;

    double fieldVelocityX = plannedVelocityX + correctionX;
    double fieldVelocityY = plannedVelocityY + correctionY;
    double turnSpeed = plannedTurnSpeed + turnCorrection;

    double linearSpeed = Math.hypot(fieldVelocityX, fieldVelocityY);
    if (maxLinearVelocity > 0 && linearSpeed > maxLinearVelocity) {
      double scale = maxLinearVelocity / linearSpeed;
      fieldVelocityX *= scale;
      fieldVelocityY *= scale;
    }
    if (maxAngularVelocity > 0) {
      turnSpeed = Math.clamp(turnSpeed, -maxAngularVelocity, maxAngularVelocity);
    }

    return new double[] {fieldVelocityX, fieldVelocityY, turnSpeed};
  }

  public boolean atReference(
      double robotX,
      double robotY,
      double robotHeading,
      double goalX,
      double goalY,
      double goalHeading) {
    double distanceToGoal = distance(robotX, robotY, goalX, goalY);
    double headingError = headingError(robotHeading, goalHeading);

    boolean closeEnoughToGoal = distanceToGoal <= translationTolerance;
    boolean facingGoalHeading = Math.abs(headingError) <= rotationTolerance;

    if (!closeEnoughToGoal) {
      return false;
    }

    if (!facingGoalHeading) {
      return false;
    }

    return true;
  }

  private static double distance(double fromX, double fromY, double toX, double toY) {
    double deltaX = toX - fromX;
    double deltaY = toY - fromY;

    return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
  }

  private static boolean isFinite(double... values) {
    for (double value : values) {
      if (!Double.isFinite(value)) {
        return false;
      }
    }
    return true;
  }

  private static double headingError(double robotHeading, double goalHeading) {
    // keeps the angle between -pi and pi so it turns the short way and doesnt spin around
    // (a while loop here would never end if a heading came in as infinity)
    return Math.IEEEremainder(goalHeading - robotHeading, 2 * Math.PI);
  }
}
