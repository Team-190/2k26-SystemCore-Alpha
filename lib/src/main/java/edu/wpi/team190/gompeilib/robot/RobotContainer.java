package edu.wpi.team190.gompeilib.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

public interface RobotContainer {

  public default void robotPeriodic() {}

  public default Command getAutonomousCommand() {
    return Commands.none();
  }
}
