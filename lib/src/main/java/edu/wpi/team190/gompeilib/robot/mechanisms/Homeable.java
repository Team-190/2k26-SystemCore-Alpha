package edu.wpi.team190.gompeilib.robot.mechanisms;

import org.wpilib.command2.Command;

/** A mechanism that finds its position by driving into a hard stop. */
public interface Homeable extends Mechanism {
  /**
   * Returns the mechanism's homing.
   *
   * @return the homing
   * @throws IllegalStateException if the mechanism wasn't configured to home
   */
  CurrentHoming<?> homing();

  /**
   * Returns a command that homes the mechanism.
   *
   * @return the command
   * @throws IllegalStateException if the mechanism wasn't configured to home
   */
  default Command home() {
    return homing().command();
  }
}
