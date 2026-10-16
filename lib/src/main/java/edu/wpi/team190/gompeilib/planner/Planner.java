package edu.wpi.team190.gompeilib.planner;

import java.util.function.Consumer;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class Planner {

  private static Consumer<ChassisVelocities> driveCommand;

  public static void setDriveCommand(Consumer<ChassisVelocities> newDriveCommand) {
    driveCommand = newDriveCommand;
  }

  public static Command followPath(Plan plan, Supplier<Pose2d> robotPose) {
    if (driveCommand == null) {
      System.out.println("DriveCommand is set to null in planner class");
      return null;
    }
    Solver solver = new Solver();

    Command command = Commands.none();
    // i think i need to redo ts cuz at refrence amkes not sense jsut make 1 command that runs each
    // state at time interval ts might be a bit harder then we initallyt thought
    for (State state : plan.getPlan()) {
      command =
          command
              .andThen(
                  Commands.run(
                          () -> {
                            driveCommand.accept(solver.calculate(robotPose.get(), state));
                          })
                      .until(
                          () -> {
                            boolean atReference = solver.atReference(robotPose.get(), state);
                            return atReference;
                          }))
              .finallyDo(
                  () -> {
                    driveCommand.accept(new ChassisVelocities());
                  });
    }
    return command;
  }
}
