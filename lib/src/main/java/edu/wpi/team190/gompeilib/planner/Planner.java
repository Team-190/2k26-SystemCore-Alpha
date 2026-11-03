package edu.wpi.team190.gompeilib.planner;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;

public class Planner {

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
              PlannerTelemetry.logStatus("Running", true);
              PlannerTelemetry.logCommands(0, "", 0);
              PlannerTelemetry.logPlan(plan);
            })
        .andThen(
            Commands.run(
                    () -> {
                      PlannerTelemetry.logIsRunning(true);
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
                        double exactIndex = planTime * Path.RATE;
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
                        PlannerTelemetry.logTiming(
                            time,
                            skippedTime[0],
                            lastTime[0] < 0 ? 0 : previousPlanTime - lastTime[0],
                            planTime - previousPlanTime);
                        ChassisVelocities output =
                            target.velocity().toRobotRelative(pose.getRotation());
                        driveCommand.accept(output);
                        PlannerTelemetry.logTarget(
                            pose,
                            target,
                            output,
                            path,
                            pathIndex,
                            planTime,
                            distanceToPathEnd,
                            lastIndex);

                        // run any command whose time we passed since the last loop
                        for (var entry : plan.getCommandMap().entrySet()) {
                          if (entry.getKey() > lastTime[0] && entry.getKey() <= planTime) {
                            CommandScheduler.getInstance().schedule(entry.getValue());
                            triggeredCommands[0]++;
                            PlannerTelemetry.logCommands(
                                triggeredCommands[0], entry.getValue().getName(), entry.getKey());
                          }
                        }
                        lastTime[0] = planTime;
                      } catch (Exception e) {
                        System.out.println(e);
                      }
                    },
                    requirements)
                .until(
                    () -> {
                      // only done once the plan time is over AAND we actually got to the end
                      boolean planTimeOver =
                          (timer.get() + skippedTime[0]) * Path.RATE >= lastIndex;
                      boolean atEnd = true; // TODO: check if actually at the end of path
                      PlannerTelemetry.logCompletion(
                          planTimeOver, atEnd, finalState.pose(), robotPose.get());
                      return planTimeOver && atEnd;
                    }))
        .finallyDo(
            interrupted -> {
              PlannerTelemetry.logStatus(interrupted ? "Interrupted" : "Completed", false);
              PlannerTelemetry.logOutput(new ChassisVelocities());
              driveCommand.accept(new ChassisVelocities(0, 0, 0));
            });
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
    PlannerTelemetry.logFieldRelativeVelocity(
        new ChassisVelocities(solved[0], solved[1], solved[2]));
    PlannerTelemetry.logFieldRelativeCorrection(
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
