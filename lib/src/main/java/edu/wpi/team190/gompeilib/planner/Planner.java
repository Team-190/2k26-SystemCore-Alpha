package edu.wpi.team190.gompeilib.planner;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;

public class Planner {

  private static final int RADIUS_CIRCLE_POINTS = 67;

  private static Consumer<ChassisVelocities> driveCommand;

  public static void setDriveCommand(Consumer<ChassisVelocities> newDriveCommand) {
    driveCommand = newDriveCommand;
  }

  public static Command followPath(Plan plan, Supplier<Pose2d> robotPose) {
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

    int lastIndex = states.size() - 1;
    State finalState = states.get(lastIndex);
    // when we run out of states just sit at the last pose w no velocity so it dont fly past it
    State holdAtEnd = new State(finalState.pose(), new ChassisVelocities(0, 0, 0), 0);

    return Commands.runOnce(
            () -> {
              timer.restart();
              lastTime[0] = -1;
              skippedTime[0] = 0;
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
                      if (pose.getTranslation().getDistance(path.getEndPose().getTranslation())
                          < path.getEndRadius()) {
                        planTime = Math.max(planTime, plan.getPathEndTime(pathIndex));
                        skippedTime[0] = planTime - time;
                      }

                      // each state is 1 / RATE seconds after the last, so find where we are
                      double exactIndex = planTime * Path.RATE;
                      int currentIndex = (int) exactIndex;

                      State target;
                      if (currentIndex < lastIndex) {
                        // blend between states
                        State before = states.get(currentIndex);
                        State after = states.get(currentIndex + 1);
                        double fraction = exactIndex - currentIndex;
                        target =
                            new State(
                                before.pose().interpolate(after.pose(), fraction),
                                before.velocity().interpolate(after.velocity(), fraction),
                                0);
                      } else {
                        target = holdAtEnd;
                      }
                      ChassisVelocities output = calculate(solver, pose, target);
                      driveCommand.accept(output);
                      logTarget(pose, target, output, path, pathIndex, planTime);

                      // run any command whose time we passed since the last loop
                      for (var entry : plan.getCommandMap().entrySet()) {
                        if (entry.getKey() > lastTime[0] && entry.getKey() <= planTime) {
                          CommandScheduler.getInstance().schedule(entry.getValue());
                        }
                      }
                      lastTime[0] = planTime;
                    })
                .until(
                    () -> {
                      // only done once the plan time is over AAND we actually got to the end
                      boolean planTimeOver =
                          (timer.get() + skippedTime[0]) * Path.RATE >= lastIndex;
                      boolean atEnd = atReference(solver, robotPose.get(), finalState);
                      return planTimeOver && atEnd;
                    }))
        .finallyDo(
            () -> {
              Logger.recordOutput("GompeiPlanner/IsRunning", false);
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
      pathStartTime = plan.getPathEndTime(i);
    }
    Logger.recordOutput(
        "GompeiPlanner/Plan/Trajectory", plan.getAsPose2dList().toArray(new Pose2d[0]));
    Logger.recordOutput("GompeiPlanner/Plan/Waypoints", waypoints);
    Logger.recordOutput("GompeiPlanner/Plan/StartSpeeds", startSpeeds);
    Logger.recordOutput("GompeiPlanner/Plan/EndSpeeds", endSpeeds);
    Logger.recordOutput("GompeiPlanner/Plan/MaxVels", maxVels);
    Logger.recordOutput("GompeiPlanner/Plan/MaxAccels", maxAccels);
    Logger.recordOutput("GompeiPlanner/Plan/EndRadii", endRadii);
    Logger.recordOutput("GompeiPlanner/Plan/Duration", (states.size() - 1) / Path.RATE);
  }

  private static void logTarget(
      Pose2d robotPose,
      State target,
      ChassisVelocities output,
      Path path,
      int pathIndex,
      double planTime) {
    Logger.recordOutput("GompeiPlanner/Target/Pose", target.pose());
    Logger.recordOutput("GompeiPlanner/Target/Velocity", target.velocity());
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
