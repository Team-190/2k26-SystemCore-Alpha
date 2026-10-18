package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * A driving behavior a joystick drive runs while its trigger is held, such as facing a heading or
 * slow mode. See {@link DriveRequest} for how several active behaviors combine.
 */
@FunctionalInterface
public interface DriveBehavior {
  /**
   * The drive's measured state this loop.
   *
   * @param heading the robot's field heading
   * @param measuredVelocities the robot-relative chassis velocities the modules measure
   */
  record Context(Rotation2d heading, ChassisVelocities measuredVelocities) {}

  /** Adjusts this loop's request. Runs every loop the behavior's trigger is held. */
  void apply(Context context, DriveRequest request);

  /** Runs once, before {@link #apply}, on the loop the behavior's trigger becomes held. */
  default void engage(Context context) {}
}
