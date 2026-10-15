package edu.wpi.team190.gompeilib.planner;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class Planner {

  public static Consumer<ChassisVelocities> driveCommand;

  public static void setDriveCommand(Consumer<ChassisVelocities> newDriveCommand) {
    driveCommand = newDriveCommand;
  }

  public static Command followPath(Plans plan, Supplier<Pose2d> robotPose) {
    if (driveCommand == null) {
      System.out.println("DriveCommand is set to null in planner class");
      return null;
    }
    Solver solver = new Solver();

    Command command = Commands.none();

    for (State state : plan) {
      Pose2d pose = robotPose.get();
      command =
          command
              .andThen(
                  Commands.run(
                          () -> {
                            driveCommand.accept(
                                solver.calculate(
                                    robotPose.get(), state));
                          })
                      .until(
                          () -> {
                            boolean atReference =
                                solver.atReference(robotPose.get(), path, time);
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
