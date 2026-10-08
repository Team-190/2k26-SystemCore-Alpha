package edu.wpi.team190.gompeilib.planner;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class Planner {

  public static PlannerControllerConfig config;
  public static Consumer<ChassisVelocities> driveCommand;

  public static void setDriveCommand(Consumer<ChassisVelocities> newDriveCommand) {
    driveCommand = newDriveCommand;
  }

  public static void setConfig(PlannerControllerConfig newConfig) {
    config = newConfig;
  }

  public static Command followPath(ArrayList<PathNode> path, Supplier<Pose2d> robotPose) {
    if (config == null) {
      System.out.println("Config is set to null in planner class");
      return null;
    }
    if (driveCommand == null) {
      System.out.println("DriveCommand is set to null in planner class");
      return null;
    }

    PlannerDriveController driveController = new PlannerDriveController(config);

    Command command = Commands.none();

    for (PathNode node : path) {
      Pose2d targetPose =
          new Pose2d(
              node.translation.isPresent()
                  ? node.translation.get()
                  : robotPose.get().getTranslation(),
              node.rotation.isPresent() ? node.rotation.get() : robotPose.get().getRotation());
      command =
          command.andThen(
              Commands.run(
                      () ->
                          driveCommand.accept(
                              driveController.calculate(
                                  robotPose.get(), targetPose, node.desiredLinearVelocity)))
                  .until(driveController::atReference));
    }
    return command;
  }
}
