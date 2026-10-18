package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands;

import static edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands.DriveCommandsTestSupport.TOLERANCE;
import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands.DriveRequest.Axis;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class DriveRequestTest {
  private static DriveRequest request() {
    return new DriveRequest(
        MetersPerSecond.of(1.0), MetersPerSecond.of(2.0), RadiansPerSecond.of(3.0));
  }

  @Test
  public void testStartsFromStickInputFieldRelative() {
    DriveRequest request = request();
    assertEquals(1.0, request.vx().in(MetersPerSecond), TOLERANCE);
    assertEquals(2.0, request.vy().in(MetersPerSecond), TOLERANCE);
    assertEquals(3.0, request.omega().in(RadiansPerSecond), TOLERANCE);
    assertTrue(request.isFieldRelative());
    for (Axis axis : Axis.values()) assertFalse(request.isOverridden(axis));
  }

  @Test
  public void testFirstOverrideOnAnAxisWins() {
    DriveRequest request = request();

    assertTrue(request.overrideX(MetersPerSecond.of(-1.0)));
    assertFalse(request.overrideX(MetersPerSecond.of(-5.0)));
    assertTrue(request.overrideY(MetersPerSecond.of(-2.0)));
    assertFalse(request.overrideY(MetersPerSecond.of(-5.0)));
    assertTrue(request.overrideOmega(RadiansPerSecond.of(-3.0)));
    assertFalse(request.overrideOmega(RadiansPerSecond.of(-5.0)));

    assertEquals(-1.0, request.vx().in(MetersPerSecond), TOLERANCE);
    assertEquals(-2.0, request.vy().in(MetersPerSecond), TOLERANCE);
    assertEquals(-3.0, request.omega().in(RadiansPerSecond), TOLERANCE);
    for (Axis axis : Axis.values()) assertTrue(request.isOverridden(axis));
  }

  @Test
  public void testScalesOnlyStickInput() {
    DriveRequest request = request();
    request.scaleInput(0.5, 0.25);
    assertEquals(0.5, request.vx().in(MetersPerSecond), TOLERANCE);
    assertEquals(1.0, request.vy().in(MetersPerSecond), TOLERANCE);
    assertEquals(0.75, request.omega().in(RadiansPerSecond), TOLERANCE);

    DriveRequest overridden = request();
    overridden.overrideX(MetersPerSecond.of(1.0));
    overridden.overrideY(MetersPerSecond.of(2.0));
    overridden.overrideOmega(RadiansPerSecond.of(3.0));
    overridden.scaleInput(0.5, 0.5);
    assertEquals(1.0, overridden.vx().in(MetersPerSecond), TOLERANCE);
    assertEquals(2.0, overridden.vy().in(MetersPerSecond), TOLERANCE);
    assertEquals(3.0, overridden.omega().in(RadiansPerSecond), TOLERANCE);
  }

  @Test
  public void testConvertsFromTheDriverFrame() {
    ChassisVelocities velocities = request().toRobotRelative(Rotation2d.fromDegrees(90.0));
    // Facing +y in the driver's frame, driving +x in that frame is driving right for the robot
    assertEquals(2.0, velocities.vx, TOLERANCE);
    assertEquals(-1.0, velocities.vy, TOLERANCE);
    assertEquals(3.0, velocities.omega, TOLERANCE);
  }

  @Test
  public void testRobotRelativeIgnoresTheHeading() {
    DriveRequest request = request();
    request.setRobotRelative();
    assertFalse(request.isFieldRelative());

    ChassisVelocities velocities = request.toRobotRelative(Rotation2d.fromDegrees(90.0));
    assertEquals(1.0, velocities.vx, TOLERANCE);
    assertEquals(2.0, velocities.vy, TOLERANCE);
    assertEquals(3.0, velocities.omega, TOLERANCE);
  }
}
