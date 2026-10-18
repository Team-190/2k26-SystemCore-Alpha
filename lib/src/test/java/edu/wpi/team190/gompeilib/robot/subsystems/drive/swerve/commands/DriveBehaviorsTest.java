package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands;

import static edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands.DriveCommandsTestSupport.TOLERANCE;
import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands.DriveRequest.Axis;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class DriveBehaviorsTest extends GompeiLibTestBase {
  private static final DriveBehavior.Context CONTEXT =
      new DriveBehavior.Context(Rotation2d.ZERO, new ChassisVelocities());

  private static DriveRequest request() {
    return new DriveRequest(
        MetersPerSecond.of(1.0), MetersPerSecond.of(2.0), RadiansPerSecond.of(3.0));
  }

  @Test
  public void testOverridesEachAxis() {
    DriveRequest request = request();
    DriveBehaviors.overrideX(() -> MetersPerSecond.of(-1.0)).apply(CONTEXT, request);
    DriveBehaviors.overrideY(() -> MetersPerSecond.of(-2.0)).apply(CONTEXT, request);
    DriveBehaviors.overrideOmega(() -> RadiansPerSecond.of(-3.0)).apply(CONTEXT, request);

    assertEquals(-1.0, request.vx().in(MetersPerSecond), TOLERANCE);
    assertEquals(-2.0, request.vy().in(MetersPerSecond), TOLERANCE);
    assertEquals(-3.0, request.omega().in(RadiansPerSecond), TOLERANCE);
  }

  @Test
  public void testScalesWithOneFactor() {
    DriveRequest request = request();
    DriveBehaviors.scale(() -> 0.5).apply(CONTEXT, request);
    assertEquals(0.5, request.vx().in(MetersPerSecond), TOLERANCE);
    assertEquals(1.0, request.vy().in(MetersPerSecond), TOLERANCE);
    assertEquals(1.5, request.omega().in(RadiansPerSecond), TOLERANCE);
  }

  @Test
  public void testScalesLinearAndAngularSeparately() {
    DriveRequest request = request();
    DriveBehaviors.scale(() -> 0.5, () -> 0.0).apply(CONTEXT, request);
    assertEquals(0.5, request.vx().in(MetersPerSecond), TOLERANCE);
    assertEquals(0.0, request.omega().in(RadiansPerSecond), TOLERANCE);
  }

  @Test
  public void testDrivesRobotRelative() {
    DriveRequest request = request();
    DriveBehaviors.robotRelative().apply(CONTEXT, request);
    assertFalse(request.isFieldRelative());
  }

  @Test
  public void testFacesAHeadingWithAndWithoutFeedforward() {
    var constants = DriveCommandsTestSupport.constants();

    DriveRequest plain = request();
    DriveBehavior face = DriveBehaviors.faceHeading(constants, () -> Rotation2d.fromDegrees(90.0));
    assertInstanceOf(FaceHeading.class, face);
    face.apply(CONTEXT, plain);
    assertTrue(plain.isOverridden(Axis.OMEGA));
    assertTrue(plain.omega().in(RadiansPerSecond) > 0.0);

    DriveRequest withFeedforward = request();
    DriveBehaviors.faceHeading(constants, () -> Rotation2d.ZERO, () -> RadiansPerSecond.of(0.7))
        .apply(CONTEXT, withFeedforward);
    // Already at the heading, so only the feedforward is commanded
    assertEquals(0.7, withFeedforward.omega().in(RadiansPerSecond), TOLERANCE);
  }
}
