package edu.wpi.team190.gompeilib.planner;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;

public class Planner {

  private static final int RADIUS_CIRCLE_POINTS = 67;

  public static Command followPath(
      Plan plan,
      Supplier<Pose2d> robotPose,
      Consumer<ChassisVelocities> driveCommand,
      Subsystem... requirements) {
    if (driveCommand == null) {
      DriverStationErrors.reportError(
          "GompeiPlanner drive command not set, call Planner.setDriveCommand()", false);
      return Commands.none();
    }
    ArrayList<State> states = plan.getPlan();
    if (states.isEmpty()) {
      DriverStationErrors.reportError(
          "GompeiPlanner was given an empty plan, nothing to follow", false);
      return Commands.none();
    }
    Solver solver = new Solver();
    Timer timer = new Timer();
    double[] lastTime = {-1};
    double[] skippedTime = {0};
    int[] triggeredCommands = {0};

    int lastIndex = states.size() - 1;
    State finalState = states.get(lastIndex);
    return Commands.runOnce(
            () -> {
              timer.restart();
              lastTime[0] = -1;
              skippedTime[0] = 0;
              triggeredCommands[0] = 0;
              Logger.recordOutput("GompeiPlanner/Status", "Running");
              Logger.recordOutput("GompeiPlanner/IsRunning", true);
              Logger.recordOutput("GompeiPlanner/Commands/TriggeredCount", 0);
              Logger.recordOutput("GompeiPlanner/Commands/LastName", "");
              logPlan(plan);
            })
        .andThen(
            Commands.run(
                    () -> {
                      Logger.recordOutput("GompeiPlanner/IsRunning", true);
                      Pose2d pose = robotPose.get();
                      double time = timer.get();
                      double planTime = time + skippedTime[0];

                      // close enough to the end of this path so just kindaa jump to the next one
                      int pathIndex = plan.pathIndexAt(planTime);
                      Path path = plan.getPath(pathIndex);
                      double previousPlanTime = planTime;
                      double distanceToPathEnd =
                          pose.getTranslation().getDistance(path.getEndPose().getTranslation());
                      boolean insideEndRadius = distanceToPathEnd < path.getEndRadius();
                      if (insideEndRadius) {
                        planTime = Math.max(planTime, plan.getPathEndTime(pathIndex));
                        skippedTime[0] = planTime - time;
                      }

                      // each state is 1 / RATE seconds after the last, so find where we are
                      try {
                        double sampleRateHz = Path.getSampleRateHz();
                        double exactIndex = planTime * sampleRateHz;
                        int currentIndex = (int) exactIndex;
                        State before = states.get(currentIndex);
                        State target;
                        State after;
                        after = states.get(currentIndex + 1);
                        double fraction = exactIndex - currentIndex;
                        target =
                            new State(
                                before.pose().interpolate(after.pose(), fraction),
                                before.velocity().interpolate(after.velocity(), fraction),
                                0);
                        Logger.recordOutput("GompeiPlanner/Timing/Elapsed", time);
                        Logger.recordOutput("GompeiPlanner/Timing/Skipped", skippedTime[0]);
                        Logger.recordOutput(
                            "GompeiPlanner/Timing/LoopDelta",
                            lastTime[0] < 0 ? 0 : previousPlanTime - lastTime[0]);
                        Logger.recordOutput(
                            "GompeiPlanner/Timing/Jump", planTime - previousPlanTime);
                        Logger.recordOutput(
                            "GompeiPlanner/Target/InsideEndRadius", insideEndRadius);
                        Logger.recordOutput(
                            "GompeiPlanner/Target/DistanceToPathEnd", distanceToPathEnd);
                        Logger.recordOutput(
                            "GompeiPlanner/Target/StateIndex",
                            Math.min(planTime * sampleRateHz, lastIndex));
                        Logger.recordOutput(
                            "GompeiPlanner/Target/AtFinalSample",
                            planTime * sampleRateHz >= lastIndex);
                        Logger.recordOutput(
                            "GompeiPlanner/Target/SampledPathIndex", plan.pathIndexAt(planTime));
                        ChassisVelocities output =
                            target.velocity().toRobotRelative(pose.getRotation());
                        driveCommand.accept(output);
                        logTarget(pose, target, output, path, pathIndex, planTime);

                        // run any command whose time we passed since the last loop
                        for (var entry : plan.getCommandMap().entrySet()) {
                          if (entry.getKey() > lastTime[0] && entry.getKey() <= planTime) {
                            CommandScheduler.getInstance().schedule(entry.getValue());
                            triggeredCommands[0]++;
                            Logger.recordOutput(
                                "GompeiPlanner/Commands/LastName", entry.getValue().getName());
                            Logger.recordOutput("GompeiPlanner/Commands/LastTime", entry.getKey());
                          }
                        }
                        Logger.recordOutput(
                            "GompeiPlanner/Commands/TriggeredCount", triggeredCommands[0]);
                        lastTime[0] = planTime;
                      } catch (Exception e) {
                        System.out.println(e);
                      }
                    },
                    requirements)
                .until(
                    () -> {
                      // only done once the plan time is over AAND we actually got to the end
                      double sampleRateHz = Path.getSampleRateHz();
                      boolean planTimeOver =
                          (timer.get() + skippedTime[0]) * sampleRateHz >= lastIndex;
                      boolean atEnd = true; // TODO: check if actually at the end of path
                      Logger.recordOutput("GompeiPlanner/Completion/PlanTimeOver", planTimeOver);
                      Logger.recordOutput("GompeiPlanner/Completion/AtReference", atEnd);
                      Logger.recordOutput(
                          "GompeiPlanner/Completion/TranslationError",
                          robotPose
                              .get()
                              .getTranslation()
                              .getDistance(finalState.pose().getTranslation()));
                      Logger.recordOutput(
                          "GompeiPlanner/Completion/HeadingError",
                          finalState
                              .pose()
                              .getRotation()
                              .minus(robotPose.get().getRotation())
                              .getRadians());
                      return planTimeOver && atEnd;
                    }))
        .finallyDo(
            interrupted -> {
              Logger.recordOutput(
                  "GompeiPlanner/Status", interrupted ? "Interrupted" : "Completed");
              Logger.recordOutput("GompeiPlanner/IsRunning", false);
              Logger.recordOutput(
                  "GompeiPlanner/Output/RobotRelativeVelocity", new ChassisVelocities());
              driveCommand.accept(new ChassisVelocities(0, 0, 0));
            });
  }

  private static void logPlan(Plan plan) {
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
    Logger.recordOutput("GompeiPlanner/Plan/SampleRateHz", Path.getSampleRateHz());
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
    Logger.recordOutput("GompeiPlanner/Plan/Duration", (states.size() - 1) / Path.getSampleRateHz());
  }

  private static void logTarget(
      Pose2d robotPose,
      State target,
      ChassisVelocities output,
      Path path,
      int pathIndex,
      double planTime) {
    Logger.recordOutput("GompeiPlanner/Robot/Pose", robotPose);
    Logger.recordOutput("GompeiPlanner/Target/Pose", target.pose());
    Logger.recordOutput("GompeiPlanner/Error/X", target.pose().getX() - robotPose.getX());
    Logger.recordOutput("GompeiPlanner/Error/Y", target.pose().getY() - robotPose.getY());
    Logger.recordOutput("GompeiPlanner/Target/Velocity", target.velocity());
    Logger.recordOutput("GompeiPlanner/Target/EndVelocity", path.getEndVelocity());
    Logger.recordOutput(
        "GompeiPlanner/Target/Speed", Math.hypot(target.velocity().vx, target.velocity().vy));
    Logger.recordOutput("GompeiPlanner/Target/PathIndex", pathIndex);
    Logger.recordOutput("GompeiPlanner/Target/PathEndPose", path.getEndPose());
    Logger.recordOutput("GompeiPlanner/Target/EndRadius", path.getEndRadius());
    Logger.recordOutput(
        "GompeiPlanner/Target/EndRadiusCircle", circle(path.getEndPose(), path.getEndRadius()));
    Logger.recordOutput("GompeiPlanner/PlanTime", planTime);
    Logger.recordOutput("GompeiPlanner/Output/RobotRelativeVelocity", output);
    Logger.recordOutput(
        "GompeiPlanner/Error/Translation",
        robotPose.getTranslation().getDistance(target.pose().getTranslation()));
    Logger.recordOutput(
        "GompeiPlanner/Error/Rotation",
        target.pose().getRotation().minus(robotPose.getRotation()).getRadians());
  }

  private static double speedAt(ArrayList<State> states, double time) {
    int index = Math.min((int) Math.round(time * Path.getSampleRateHz()), states.size() - 1);
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

  private static ChassisVelocities calculate(Solver solver, Pose2d robotPose, State goal) {
    double[] solved =
        solver.calculate(
            robotPose.getX(),
            robotPose.getY(),
            robotPose.getRotation().getRadians(),
            goal.pose().getX(),
            goal.pose().getY(),
            goal.pose().getRotation().getRadians(),
            goal.velocity().vx,
            goal.velocity().vy,
            goal.velocity().omega);
    Logger.recordOutput(
        "GompeiPlanner/Output/FieldRelativeVelocity",
        new ChassisVelocities(solved[0], solved[1], solved[2]));
    Logger.recordOutput(
        "GompeiPlanner/Output/FieldRelativeCorrection",
        new ChassisVelocities(
            solved[0] - goal.velocity().vx,
            solved[1] - goal.velocity().vy,
            solved[2] - goal.velocity().omega));
    return new ChassisVelocities(solved[0], solved[1], solved[2])
        .toRobotRelative(robotPose.getRotation());
  }

  private static boolean atReference(Solver solver, Pose2d robotPose, State goal) {
    return solver.atReference(
        robotPose.getX(),
        robotPose.getY(),
        robotPose.getRotation().getRadians(),
        goal.pose().getX(),
        goal.pose().getY(),
        goal.pose().getRotation().getRadians());
  }
}
