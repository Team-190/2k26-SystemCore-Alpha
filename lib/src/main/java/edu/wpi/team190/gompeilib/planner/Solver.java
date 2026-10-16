package edu.wpi.team190.gompeilib.planner;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

// solves ACTUAL chassis velocities based on current state and goal state
public class Solver {
  private static final double DEFAULT_TRANSLATION_GAIN = 1.0;
  private static final double DEFAULT_ROTATION_GAIN = 4.0;
  private static final double DEFAULT_TRANSLATION_TOLERANCE = 0.05; // meters
  private static final double DEFAULT_ROTATION_TOLERANCE = Math.toRadians(2);

  private final double translationGain;
  private final double rotationGain;
  private final double translationTolerance;
  private final double rotationTolerance;

  public Solver() {
    this(
        DEFAULT_TRANSLATION_GAIN,
        DEFAULT_ROTATION_GAIN,
        DEFAULT_TRANSLATION_TOLERANCE,
        DEFAULT_ROTATION_TOLERANCE);
  }

  public Solver(
      double translationGain,
      double rotationGain,
      double translationTolerance,
      double rotationTolerance) {
    this.translationGain = translationGain;
    this.rotationGain = rotationGain;
    this.translationTolerance = translationTolerance;
    this.rotationTolerance = rotationTolerance;
  }

  // needs to be chnaged as it uses wpilib stuff idk how co processor works but maybe we submit it
  // to network tables or smth like that or just have it return a few values then  onstruct a chasis
  // velocity using it
  public ChassisVelocities calculate(Pose2d robotPose, State goal) {
    double[] solved =
        calculate(
            robotPose.getX(),
            robotPose.getY(),
            robotPose.getRotation().getRadians(),
            goal.pose().getX(),
            goal.pose().getY(),
            goal.pose().getRotation().getRadians(),
            goal.velocity().vx,
            goal.velocity().vy,
            goal.velocity().omega);
    return new ChassisVelocities(solved[0], solved[1], solved[2]);
  }

  public boolean atReference(Pose2d robotPose, State goal) {
    return atReference(
        robotPose.getX(),
        robotPose.getY(),
        robotPose.getRotation().getRadians(),
        goal.pose().getX(),
        goal.pose().getY(),
        goal.pose().getRotation().getRadians());
  }

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
    double errorX = goalX - robotX;
    double errorY = goalY - robotY;
    double headingError = headingError(robotHeading, goalHeading);

    double correctionX = translationGain * errorX;
    double correctionY = translationGain * errorY;
    double turnCorrection = rotationGain * headingError;

    double fieldVelocityX = plannedVelocityX + correctionX;
    double fieldVelocityY = plannedVelocityY + correctionY;
    double turnSpeed = plannedTurnSpeed + turnCorrection;

    double robotVelocityX = fieldToRobotX(fieldVelocityX, fieldVelocityY, robotHeading);
    double robotVelocityY = fieldToRobotY(fieldVelocityX, fieldVelocityY, robotHeading);

    return new double[] {robotVelocityX, robotVelocityY, turnSpeed};
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

  private static double headingError(double robotHeading, double goalHeading) {
    double error = goalHeading - robotHeading;

    // keeps the angle between -pi and pi so it turns the short way and doesnt spin around
    while (error > Math.PI) {
      error -= 2 * Math.PI;
    }

    while (error < -Math.PI) {
      error += 2 * Math.PI;
    }

    return error;
  }

  private static double fieldToRobotX(
      double fieldVelocityX, double fieldVelocityY, double robotHeading) {
    double cos = Math.cos(robotHeading);
    double sin = Math.sin(robotHeading);

    return fieldVelocityX * cos + fieldVelocityY * sin;
  }

  private static double fieldToRobotY(
      double fieldVelocityX, double fieldVelocityY, double robotHeading) {
    double cos = Math.cos(robotHeading);
    double sin = Math.sin(robotHeading);

    return -fieldVelocityX * sin + fieldVelocityY * cos;
  }
}
