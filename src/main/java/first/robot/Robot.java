// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import first.robot.v3_Horse_CV2_TR.subsystems.V3_Horse_CV2_TRRobotContainer;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.framework.TimedRobot;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private final V3_Horse_CV2_TRRobotContainer robotContainer;

  public Robot() {
    robotContainer = new V3_Horse_CV2_TRRobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Logger.recordOutput("RobotPose", robotContainer.getPose());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void utilityInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void utilityPeriodic() {}

  @Override
  public void utilityExit() {}
}
