package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class DriveBehaviorTest {
  @Test
  public void testEngageDefaultsToNothing() {
    DriveRequest request =
        new DriveRequest(
            MetersPerSecond.of(1.0), MetersPerSecond.of(0.0), RadiansPerSecond.of(0.0));
    DriveBehavior behavior = (context, r) -> r.setRobotRelative();
    DriveBehavior.Context context =
        new DriveBehavior.Context(
            Rotation2d.fromDegrees(90.0), new ChassisVelocities(1.0, 2.0, 3.0));

    behavior.engage(context);
    assertTrue(request.isFieldRelative());
    behavior.apply(context, request);
    assertFalse(request.isFieldRelative());
  }

  @Test
  public void testContextCarriesTheMeasuredState() {
    ChassisVelocities measured = new ChassisVelocities(1.0, 2.0, 3.0);
    DriveBehavior.Context context =
        new DriveBehavior.Context(Rotation2d.fromDegrees(90.0), measured);
    assertEquals(Rotation2d.fromDegrees(90.0), context.heading());
    assertSame(measured, context.measuredVelocities());
  }
}
