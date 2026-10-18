package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands;

import static edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands.DriveCommandsTestSupport.TOLERANCE;
import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

public class FaceHeadingTest extends GompeiLibTestBase {
  private static final double LOOP_SECONDS = 0.02;
  private static final double HEADING_TOLERANCE_RAD = Math.toRadians(1.5);

  private MockTunableBackend backend;
  private double headingRad;
  private double omegaRadPerSec;

  @Override
  protected boolean isTuningMode() {
    return true;
  }

  @BeforeEach
  public void setUp() {
    TunableRegistry.reset();
    backend = new MockTunableBackend();
    TunableRegistry.registerBackend("", backend);
    headingRad = 0.0;
    omegaRadPerSec = 0.0;
  }

  @AfterEach
  public void tearDown() {
    TunableRegistry.reset();
  }

  private static FaceHeading faceHeading(Rotation2d target) {
    return new FaceHeading(
        DriveCommandsTestSupport.autoAlign(DriveCommandsTestSupport.rotationGains("face", 4.0)),
        () -> target,
        () -> RadiansPerSecond.of(0.0));
  }

  private DriveBehavior.Context context() {
    return new DriveBehavior.Context(
        Rotation2d.fromRadians(headingRad), new ChassisVelocities(0.0, 0.0, omegaRadPerSec));
  }

  private static DriveRequest request() {
    return new DriveRequest(
        MetersPerSecond.of(0.0), MetersPerSecond.of(0.0), RadiansPerSecond.of(0.0));
  }

  /** Runs one loop, turning the simulated robot at the commanded angular velocity. */
  private void step(FaceHeading face) {
    DriveRequest request = request();
    face.apply(context(), request);
    omegaRadPerSec = request.omega().in(RadiansPerSecond);
    headingRad += omegaRadPerSec * LOOP_SECONDS;
  }

  @Test
  public void testConvergesOnTheTargetHeading() {
    FaceHeading face = faceHeading(Rotation2d.fromDegrees(90.0));
    face.engage(context());
    for (int i = 0; i < 250; i++) step(face);
    assertEquals(Math.PI / 2, headingRad, HEADING_TOLERANCE_RAD);
  }

  @Test
  public void testRestartsTheProfileFromTheCurrentHeadingWhenReengaged() {
    FaceHeading face = faceHeading(Rotation2d.fromDegrees(90.0));
    face.engage(context());
    for (int i = 0; i < 250; i++) step(face);

    // Released, then the robot is turned to face the other way before re-engaging
    headingRad = -Math.PI / 2;
    omegaRadPerSec = 0.0;
    face.engage(context());
    step(face);

    // The profile starts from -90 deg, not from the +90 deg setpoint it ended on
    assertEquals(-Math.PI / 2, face.getController().getSetpoint().position, 0.05);
  }

  @Test
  public void testYieldsToAnEarlierOverrideAndRestartsWhenItLetsGo() {
    FaceHeading face = faceHeading(Rotation2d.fromDegrees(90.0));
    face.engage(context());
    for (int i = 0; i < 250; i++) step(face);

    DriveRequest overridden = request();
    overridden.overrideOmega(RadiansPerSecond.of(1.0));
    face.apply(context(), overridden);
    assertEquals(1.0, overridden.omega().in(RadiansPerSecond), TOLERANCE);

    headingRad = 0.0;
    omegaRadPerSec = 0.0;
    step(face);
    assertEquals(0.0, face.getController().getSetpoint().position, 0.05);
  }

  @Test
  public void testHoldsZeroOutputOnceAtTheHeading() {
    FaceHeading face = faceHeading(Rotation2d.fromDegrees(0.0));
    face.engage(context());
    for (int i = 0; i < 5; i++) {
      DriveRequest request = request();
      face.apply(context(), request);
      assertEquals(0.0, request.omega().in(RadiansPerSecond), TOLERANCE);
    }
  }

  @Test
  public void testFollowsLiveRotationGainEdits() {
    FaceHeading face = faceHeading(Rotation2d.fromDegrees(90.0));
    assertEquals(4.0, face.getController().getP(), TOLERANCE);

    backend.setDouble("/face/kP", 6.0);
    TunableRegistry.update();
    assertEquals(6.0, face.getController().getP(), TOLERANCE);
    assertEquals(0.0, face.getController().getI(), TOLERANCE);
  }
}
