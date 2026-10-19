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
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;

public class Planner {

  private static Consumer<ChassisVelocities> driveCommand;

  public static void setDriveCommand(Consumer<ChassisVelocities> newDriveCommand) {
    driveCommand = newDriveCommand;
  }

  public static Command followPath(Plan plan, Supplier<Pose2d> robotPose) {
    if (driveCommand == null) {
      DriverStationErrors.reportError(
          "Planner drive command not set, call Planner.setDriveCommand()", false);
      return Commands.none();
    }
    ArrayList<State> states = plan.getPlan();
    if (states.isEmpty()) {
      DriverStationErrors.reportError("Planner was given an empty plan, nothing to follow", false);
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
            })
        .andThen(
            Commands.run(
                    () -> {
                      Logger.recordOutput("Planner/isRunning", true);
                      Pose2d[] poses = (Pose2d[]) plan.getAsPose2dList().toArray();
                      Logger.recordOutput("Planner/path", poses);
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
                      Logger.recordOutput("Planner/target", target);
                      driveCommand.accept(calculate(solver, pose, target));

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
              Logger.recordOutput("Planner/isRunning", false);
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
