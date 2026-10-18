package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class DriveSticksTest {
  @Test
  public void testExposesEachStick() {
    DriveSticks sticks = new DriveSticks(() -> 0.1, () -> -0.2, () -> 0.3);
    assertEquals(0.1, sticks.x().getAsDouble());
    assertEquals(-0.2, sticks.y().getAsDouble());
    assertEquals(0.3, sticks.omega().getAsDouble());
  }

  @Test
  public void testNoneIsCentered() {
    assertEquals(0.0, DriveSticks.NONE.x().getAsDouble());
    assertEquals(0.0, DriveSticks.NONE.y().getAsDouble());
    assertEquals(0.0, DriveSticks.NONE.omega().getAsDouble());
  }
}
