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
        "GompeiPlanner/Plan/Trajectory", plan.getAsPose2dList().toArray(new Pose2d[0]));
    Logger.recordOutput("GompeiPlanner/Plan/Waypoints", waypoints);
    Logger.recordOutput("GompeiPlanner/Plan/StateCount", states.size());
    Logger.recordOutput("GompeiPlanner/Plan/PathCount", pathCount);
    Logger.recordOutput("GompeiPlanner/Plan/SampleRateHz", Path.RATE);
    Logger.recordOutput("GompeiPlanner/Plan/PathEndTimes", endTimes);
    Logger.recordOutput("GompeiPlanner/Plan/RequestedEndVelocities", endVelocities);
    Logger.recordOutput(
        "GompeiPlanner/Plan/SampleVelocities",
        states.stream().map(State::velocity).toArray(ChassisVelocities[]::new));
    Logger.recordOutput(
        "GompeiPlanner/Plan/SampleDistances",
        states.stream().mapToDouble(State::distance).toArray());
    Logger.recordOutput(
        "GompeiPlanner/Plan/CommandTimes",
        plan.getCommandMap().keySet().stream().sorted().mapToDouble(Double::doubleValue).toArray());
    Logger.recordOutput(
        "GompeiPlanner/Plan/CommandNames",
        plan.getCommandMap().entrySet().stream()
            .sorted(Comparator.comparingDouble(entry -> entry.getKey()))
            .map(entry -> entry.getValue().getName())
            .toArray(String[]::new));
    Logger.recordOutput("GompeiPlanner/Plan/StartSpeeds", startSpeeds);
    Logger.recordOutput("GompeiPlanner/Plan/EndSpeeds", endSpeeds);
    Logger.recordOutput("GompeiPlanner/Plan/MaxVels", maxVels);
    Logger.recordOutput("GompeiPlanner/Plan/MaxAccels", maxAccels);
    Logger.recordOutput("GompeiPlanner/Plan/EndRadii", endRadii);
    Logger.recordOutput("GompeiPlanner/Plan/Duration", (states.size() - 1) / Path.RATE);
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
    Logger.recordOutput("GompeiPlanner/Target/InsideEndRadius", insideEndRadius);
    Logger.recordOutput("GompeiPlanner/Target/DistanceToPathEnd", distanceToPathEnd);
    Logger.recordOutput(
        "GompeiPlanner/Target/StateIndex", Math.min(planTime * Path.RATE, lastStateIndex));
    Logger.recordOutput(
        "GompeiPlanner/Target/AtFinalSample", planTime * Path.RATE >= lastStateIndex);
    Logger.recordOutput("GompeiPlanner/Target/SampledPathIndex", pathIndex);

    // pose values are heavy visual logs, downsample to 50 Hz
    if (shouldLogHeavyTelemetry()) {
      Logger.recordOutput("GompeiPlanner/Robot/Pose", robotPose);
      Logger.recordOutput("GompeiPlanner/Target/Pose", target.pose());
      Logger.recordOutput("GompeiPlanner/Target/EndVelocity", path.getEndVelocity());
      Logger.recordOutput("GompeiPlanner/Target/PathEndPose", path.getEndPose());
      Logger.recordOutput(
          "GompeiPlanner/Target/EndRadiusCircle", circle(path.getEndPose(), path.getEndRadius()));
    }

    Logger.recordOutput("GompeiPlanner/Error/X", target.pose().getX() - robotPose.getX());
    Logger.recordOutput("GompeiPlanner/Error/Y", target.pose().getY() - robotPose.getY());
    Logger.recordOutput("GompeiPlanner/Target/Velocity", target.velocity());
    Logger.recordOutput(
        "GompeiPlanner/Target/Speed", Math.hypot(target.velocity().vx, target.velocity().vy));
    Logger.recordOutput("GompeiPlanner/Target/PathIndex", pathIndex);
    Logger.recordOutput("GompeiPlanner/Target/EndRadius", path.getEndRadius());
    Logger.recordOutput("GompeiPlanner/PlanTime", planTime);
    // motor outputs run at full rate, so no downsampling here
    Logger.recordOutput("GompeiPlanner/Output/RobotRelativeVelocity", output);
    Logger.recordOutput(
        "GompeiPlanner/Error/Translation",
        robotPose.getTranslation().getDistance(target.pose().getTranslation()));
    Logger.recordOutput(
        "GompeiPlanner/Error/Rotation",
        target.pose().getRotation().minus(robotPose.getRotation()).getRadians());
  }

  public static void logTiming(double elapsed, double skipped, double loopDelta, double jump) {
    Logger.recordOutput("GompeiPlanner/Timing/Elapsed", elapsed);
    Logger.recordOutput("GompeiPlanner/Timing/Skipped", skipped);
    Logger.recordOutput("GompeiPlanner/Timing/LoopDelta", loopDelta);
    Logger.recordOutput("GompeiPlanner/Timing/Jump", jump);
  }

  public static void logStatus(String status, boolean isRunning) {
    Logger.recordOutput("GompeiPlanner/Status", status);
    Logger.recordOutput("GompeiPlanner/IsRunning", isRunning);
  }

  public static void logIsRunning(boolean isRunning) {
    Logger.recordOutput("GompeiPlanner/IsRunning", isRunning);
  }

  public static void logCommands(
      int triggeredCount, String lastCommandName, double lastCommandTime) {
    Logger.recordOutput("GompeiPlanner/Commands/TriggeredCount", triggeredCount);
    Logger.recordOutput("GompeiPlanner/Commands/LastName", lastCommandName);
    Logger.recordOutput("GompeiPlanner/Commands/LastTime", lastCommandTime);
  }

  public static void logOutput(ChassisVelocities robotRelativeVelocity) {
    Logger.recordOutput("GompeiPlanner/Output/RobotRelativeVelocity", robotRelativeVelocity);
  }

  public static void logFieldRelativeVelocity(ChassisVelocities fieldRelativeVelocity) {
    Logger.recordOutput("GompeiPlanner/Output/FieldRelativeVelocity", fieldRelativeVelocity);
  }

  public static void logFieldRelativeCorrection(ChassisVelocities fieldRelativeCorrection) {
    Logger.recordOutput("GompeiPlanner/Output/FieldRelativeCorrection", fieldRelativeCorrection);
  }

  public static void logCompletion(
      boolean planTimeOver, boolean atEnd, Pose2d finalPose, Pose2d robotPose) {
    Logger.recordOutput("GompeiPlanner/Completion/PlanTimeOver", planTimeOver);
    Logger.recordOutput("GompeiPlanner/Completion/AtReference", atEnd);
    Logger.recordOutput(
        "GompeiPlanner/Completion/TranslationError",
        robotPose.getTranslation().getDistance(finalPose.getTranslation()));
    Logger.recordOutput(
        "GompeiPlanner/Completion/HeadingError",
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
