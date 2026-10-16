package edu.wpi.team190.gompeilib.planner;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
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
      System.out.println("DriveCommand is set to null in planner class");
      return Commands.none();
    }
    ArrayList<State> states = plan.getPlan();
    if (states.isEmpty()) {
      return Commands.none();
    }
    Solver solver = new Solver();
    Timer timer = new Timer();

    int lastIndex = states.size() - 1;
    State finalState = states.get(lastIndex);
    // when we run out of states just sit at the last pose w no velocity so it dont fly past it
    State holdAtEnd = new State(finalState.pose(), new ChassisVelocities(0, 0, 0));

    return Commands.runOnce(timer::restart)
        .andThen(
            Commands.run(
                    () -> {
                      // each state is 1 / RATE seconds after the last, so find where we are
                      double exactIndex = timer.get() * Path.RATE;
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
                                before.velocity().interpolate(after.velocity(), fraction));
                      } else {
                        target = holdAtEnd;
                      }

                      driveCommand.accept(calculate(solver, robotPose.get(), target));

                      Command command = plan.getCommandMap().get(timer.get());
                      if (command != null) {
                        CommandScheduler.getInstance().schedule(command);
                      }
                    })
                .until(
                    () -> {
                      // only done once the plan time is over AAND we actually got to the end
                      boolean planTimeOver = timer.get() * Path.RATE >= lastIndex;
                      boolean atEnd = atReference(solver, robotPose.get(), finalState);
                      return planTimeOver && atEnd;
                    }))
        .finallyDo(
            () -> {
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
