package edu.wpi.team190.gompeilib.planner;

import java.util.ArrayList;
import java.util.Comparator;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public final class PlannerTelemetry {

  // heavy visual logs (3D circles and pose histories) are downsampled to 50 Hz so we dont flood
  // NetworkTables while motor outputs still run at full rate
  // (the end radius circle is the heavy one: it allocates 68 poses every call)
  private static final double HEAVY_LOG_INTERVAL_SECONDS = 0.02; // 50 Hz
  private static double lastHeavyLogTime = Double.NEGATIVE_INFINITY;

  private static final int RADIUS_CIRCLE_POINTS = 28;

  private static final String AKIT_TOPIC = "GompeiPlanner";
  private static final String AKIT_PLAN_TOPIC = AKIT_TOPIC + "/Plan";
  private static final String AKIT_TARGET_TOPIC = AKIT_TOPIC + "/Target";
  private static final String AKIT_ROBOT_TOPIC = AKIT_TOPIC + "/Robot";
  private static final String AKIT_ERROR_TOPIC = AKIT_TOPIC + "/Error";
  private static final String AKIT_OUTPUT_TOPIC = AKIT_TOPIC + "/Output";
  private static final String AKIT_TIMING_TOPIC = AKIT_TOPIC + "/Timing";
  private static final String AKIT_STATUS_TOPIC = AKIT_TOPIC + "/Status";
  private static final String AKIT_IS_RUNNING_TOPIC = AKIT_TOPIC + "/IsRunning";
  private static final String AKIT_COMMANDS_TOPIC = AKIT_TOPIC + "/Commands";
  private static final String AKIT_COMPLETION_TOPIC = AKIT_TOPIC + "/Completion";

  private PlannerTelemetry() {}

  // heavy visual telemetry (3D circles, pose histories) gets downsampled to 50 Hz
  // uses Logger's timestamp so the gate lines up with the replayed/simulated log timeline
  private static boolean shouldLogHeavyTelemetry() {
    double now = Logger.getTimestamp() * 1e-9; // Logger.getTimestamp() is in nanoseconds
    if (now - lastHeavyLogTime >= HEAVY_LOG_INTERVAL_SECONDS) {
      lastHeavyLogTime = now;
      return true;
    }
    return false;
  }

  public static void logPlan(Plan plan) {
    // a new plan means a new followPath, so start the downsampling gate fresh
    lastHeavyLogTime = Double.NEGATIVE_INFINITY;
    ArrayList<State> states = plan.getPlan();
    int pathCount = plan.getPathCount();
    Pose2d[] waypoints = new Pose2d[pathCount + 1];
    double[] startSpeeds = new double[pathCount];
    double[] endSpeeds = new double[pathCount];
    double[] maxVels = new double[pathCount];
    double[] maxAccels = new double[pathCount];
    double[] endRadii = new double[pathCount];
    double[] endTimes = new double[pathCount];
    ChassisVelocities[] endVelocities = new ChassisVelocities[pathCount];
    double pathStartTime = 0;
    for (int i = 0; i < pathCount; i++) {
      Path path = plan.getPath(i);
      waypoints[i] = path.getStartPose();
      waypoints[i + 1] = path.getEndPose();
      startSpeeds[i] = speedAt(states, pathStartTime);
      endSpeeds[i] = speedAt(states, plan.getPathEndTime(i));
      maxVels[i] = path.getMaxVel();
      maxAccels[i] = path.getMaxAccel();
      endRadii[i] = path.getEndRadius();
      endTimes[i] = plan.getPathEndTime(i);
      endVelocities[i] = path.getEndVelocity();
      pathStartTime = plan.getPathEndTime(i);
    }
    Logger.recordOutput(
        AKIT_PLAN_TOPIC + "/Trajectory", plan.getAsPose2dList().toArray(new Pose2d[0]));
    Logger.recordOutput(AKIT_PLAN_TOPIC + "/Waypoints", waypoints);
    Logger.recordOutput(AKIT_PLAN_TOPIC + "/StateCount", states.size());
    Logger.recordOutput(AKIT_PLAN_TOPIC + "/PathCount", pathCount);
    Logger.recordOutput(AKIT_PLAN_TOPIC + "/SampleRateHz", Path.RATE);
    Logger.recordOutput(AKIT_PLAN_TOPIC + "/PathEndTimes", endTimes);
    Logger.recordOutput(AKIT_PLAN_TOPIC + "/RequestedEndVelocities", endVelocities);
    Logger.recordOutput(
        AKIT_PLAN_TOPIC + "/SampleVelocities",
        states.stream().map(State::velocity).toArray(ChassisVelocities[]::new));
    Logger.recordOutput(
        AKIT_PLAN_TOPIC + "/SampleDistances",
        states.stream().mapToDouble(State::distance).toArray());
    Logger.recordOutput(
        AKIT_PLAN_TOPIC + "/CommandTimes",
        plan.getCommandMap().keySet().stream().sorted().mapToDouble(Double::doubleValue).toArray());
    Logger.recordOutput(
        AKIT_PLAN_TOPIC + "/CommandNames",
        plan.getCommandMap().entrySet().stream()
            .sorted(Comparator.comparingDouble(entry -> entry.getKey()))
            .map(entry -> entry.getValue().getName())
            .toArray(String[]::new));
    Logger.recordOutput(AKIT_PLAN_TOPIC + "/StartSpeeds", startSpeeds);
    Logger.recordOutput(AKIT_PLAN_TOPIC + "/EndSpeeds", endSpeeds);
    Logger.recordOutput(AKIT_PLAN_TOPIC + "/MaxVels", maxVels);
    Logger.recordOutput(AKIT_PLAN_TOPIC + "/MaxAccels", maxAccels);
    Logger.recordOutput(AKIT_PLAN_TOPIC + "/EndRadii", endRadii);
    Logger.recordOutput(AKIT_PLAN_TOPIC + "/Duration", (states.size() - 1) / Path.RATE);
  }

  public static void logTarget(
      Pose2d robotPose,
      State target,
      ChassisVelocities output,
      Path path,
      int pathIndex,
      double planTime,
      double distanceToPathEnd,
      int lastStateIndex,
      boolean insideEndRadius) {
    Logger.recordOutput(AKIT_TARGET_TOPIC + "/InsideEndRadius", insideEndRadius);
    Logger.recordOutput(AKIT_TARGET_TOPIC + "/DistanceToPathEnd", distanceToPathEnd);
    Logger.recordOutput(
        AKIT_TARGET_TOPIC + "/StateIndex", Math.min(planTime * Path.RATE, lastStateIndex));
    Logger.recordOutput(
        AKIT_TARGET_TOPIC + "/AtFinalSample", planTime * Path.RATE >= lastStateIndex);
    Logger.recordOutput(AKIT_TARGET_TOPIC + "/SampledPathIndex", pathIndex);

    // pose values are heavy visual logs, downsample to 50 Hz
    if (shouldLogHeavyTelemetry()) {
      Logger.recordOutput(AKIT_ROBOT_TOPIC + "/Pose", robotPose);
      Logger.recordOutput(AKIT_TARGET_TOPIC + "/Pose", target.pose());
      Logger.recordOutput(AKIT_TARGET_TOPIC + "/EndVelocity", path.getEndVelocity());
      Logger.recordOutput(AKIT_TARGET_TOPIC + "/PathEndPose", path.getEndPose());
      Logger.recordOutput(
          AKIT_TARGET_TOPIC + "/EndRadiusCircle", circle(path.getEndPose(), path.getEndRadius()));
    }

    Logger.recordOutput(AKIT_ERROR_TOPIC + "/X", target.pose().getX() - robotPose.getX());
    Logger.recordOutput(AKIT_ERROR_TOPIC + "/Y", target.pose().getY() - robotPose.getY);
    Logger.recordOutput(AKIT_TARGET_TOPIC + "/Velocity", target.velocity());
    Logger.recordOutput(
        AKIT_TARGET_TOPIC + "/Speed", Math.hypot(target.velocity().vx, target.velocity().vy));
    Logger.recordOutput(AKIT_TARGET_TOPIC + "/PathIndex", pathIndex);
    Logger.recordOutput(AKIT_TARGET_TOPIC + "/EndRadius", path.getEndRadius());
    Logger.recordOutput(AKIT_TOPIC + "/PlanTime", planTime);
    // motor outputs run at full rate, so no downsampling here
    Logger.recordOutput(AKIT_OUTPUT_TOPIC + "/RobotRelativeVelocity", output);
    Logger.recordOutput(
        AKIT_ERROR_TOPIC + "/Translation",
        robotPose.getTranslation().getDistance(target.pose().getTranslation()));
    Logger.recordOutput(
        AKIT_ERROR_TOPIC + "/Rotation",
        target.pose().getRotation().minus(robotPose.getRotation()).getRadians());
  }

  public static void logTiming(double elapsed, double skipped, double loopDelta, double jump) {
    Logger.recordOutput(AKIT_TIMING_TOPIC + "/Elapsed", elapsed);
    Logger.recordOutput(AKIT_TIMING_TOPIC + "/Skipped", skipped);
    Logger.recordOutput(AKIT_TIMING_TOPIC + "/LoopDelta", loopDelta);
    Logger.recordOutput(AKIT_TIMING_TOPIC + "/Jump", jump);
  }

  public static void logStatus(String status, boolean isRunning) {
    Logger.recordOutput(AKIT_STATUS_TOPIC, status);
    Logger.recordOutput(AKIT_IS_RUNNING_TOPIC, isRunning);
  }

  public static void logIsRunning(boolean isRunning) {
    Logger.recordOutput(AKIT_IS_RUNNING_TOPIC, isRunning);
  }

  public static void logCommands(
      int triggeredCount, String lastCommandName, double lastCommandTime) {
    Logger.recordOutput(AKIT_COMMANDS_TOPIC + "/TriggeredCount", triggeredCount);
    Logger.recordOutput(AKIT_COMMANDS_TOPIC + "/LastName", lastCommandName);
    Logger.recordOutput(AKIT_COMMANDS_TOPIC + "/LastTime", lastCommandTime);
  }

  public static void logOutput(ChassisVelocities robotRelativeVelocity) {
    Logger.recordOutput(AKIT_OUTPUT_TOPIC + "/RobotRelativeVelocity", robotRelativeVelocity);
  }

  public static void logFieldRelativeVelocity(ChassisVelocities fieldRelativeVelocity) {
    Logger.recordOutput(AKIT_OUTPUT_TOPIC + "/FieldRelativeVelocity", fieldRelativeVelocity);
  }

  public static void logFieldRelativeCorrection(ChassisVelocities fieldRelativeCorrection) {
    Logger.recordOutput(AKIT_OUTPUT_TOPIC + "/FieldRelativeCorrection", fieldRelativeCorrection);
  }

  public static void logCompletion(
      boolean planTimeOver, boolean atEnd, Pose2d finalPose, Pose2d robotPose) {
    Logger.recordOutput(AKIT_COMPLETION_TOPIC + "/PlanTimeOver", planTimeOver);
    Logger.recordOutput(AKIT_COMPLETION_TOPIC + "/AtReference", atEnd);
    Logger.recordOutput(
        AKIT_COMPLETION_TOPIC + "/TranslationError",
        robotPose.getTranslation().getDistance(finalPose.getTranslation()));
    Logger.recordOutput(
        AKIT_COMPLETION_TOPIC + "/HeadingError",
        finalPose.getRotation().minus(robotPose.getRotation()).getRadians());
  }

  private static double speedAt(ArrayList<State> states, double time) {
    int index = Math.min((int) Math.round(time * Path.RATE), states.size() - 1);
    ChassisVelocities velocity = states.get(index).velocity();
    return Math.hypot(velocity.vx, velocity.vy);
  }

  // this is lwk kinda cooked, but idk how to create a circle in advantage scope
  private static Pose2d[] circle(Pose2d center, double radius) {
    if (radius <= 0) {
      return new Pose2d[0];
    }
    Pose2d[] points = new Pose2d[RADIUS_CIRCLE_POINTS + 1];
    for (int i = 0; i <= RADIUS_CIRCLE_POINTS; i++) {
      Rotation2d angle = new Rotation2d(2 * Math.PI * i / RADIUS_CIRCLE_POINTS);
      points[i] =
          new Pose2d(
              center.getX() + radius * angle.getCos(),
              center.getY() + radius * angle.getSin(),
              angle);
    }
    return points;
  }
}
