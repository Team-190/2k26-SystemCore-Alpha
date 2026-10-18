package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands;

import java.util.function.DoubleSupplier;

/**
 * The driver's stick inputs for a joystick drive, each in [-1, 1].
 *
 * <p>Already signed for the drive's frame: positive {@code x} drives away from the driver, positive
 * {@code y} drives to the driver's left, and positive {@code omega} turns counterclockwise.
 *
 * @param x forward input
 * @param y leftward input
 * @param omega counterclockwise rotation input
 */
public record DriveSticks(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
  /** Sticks that are always centered, for drive commands with no driver input. */
  public static final DriveSticks NONE = new DriveSticks(() -> 0.0, () -> 0.0, () -> 0.0);
}
