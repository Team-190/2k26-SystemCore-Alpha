package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands;

import static edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands.DriveCommandsTestSupport.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDrive;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.ArgumentCaptor;
import org.mockito.MockedStatic;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class JoystickDriveTest extends GompeiLibTestBase {
  private MockedStatic<Logger> logger;
  private SwerveDrive drive;
  private Rotation2d heading;

  @BeforeEach
  public void setUp() {
    logger = mockStatic(Logger.class);
    drive = DriveCommandsTestSupport.drive();
    heading = Rotation2d.ZERO;
  }

  @AfterEach
  public void tearDown() {
    logger.close();
  }

  private static DriveSticks sticks(double x, double y, double omega) {
    return new DriveSticks(() -> x, () -> y, () -> omega);
  }

  private static DriveRequest shape(double x, double y, double omega) {
    return JoystickDrive.shape(sticks(x, y, omega), DEADBAND, MAX_LINEAR, MAX_ANGULAR);
  }

  private JoystickDrive.Builder builder(DriveSticks sticks) {
    return SwerveDriveCommands.joystickDrive(
        drive, DriveCommandsTestSupport.constants(), sticks, () -> heading);
  }

  /** Runs one loop and returns the robot-relative velocities sent to the drive. */
  private ChassisVelocities run(JoystickDrive command) {
    clearInvocations(drive);
    command.execute();
    ArgumentCaptor<ChassisVelocities> captor = ArgumentCaptor.forClass(ChassisVelocities.class);
    verify(drive).runVelocity(captor.capture());
    return captor.getValue();
  }

  @Test
  public void testDeadbandsTheSticks() {
    DriveRequest request = shape(0.05, 0.05, 0.05);
    assertEquals(0.0, request.vx().in(MetersPerSecond), TOLERANCE);
    assertEquals(0.0, request.vy().in(MetersPerSecond), TOLERANCE);
    assertEquals(0.0, request.omega().in(RadiansPerSecond), TOLERANCE);

    DriveRequest centered = shape(0.0, 0.0, 0.0);
    assertEquals(0.0, centered.vx().in(MetersPerSecond), TOLERANCE);
  }

  @Test
  public void testSquaresTheLinearMagnitudeOnly() {
    // 0.55 deadbands to 0.5; squared, that's a quarter of max speed
    DriveRequest request = shape(0.55, 0.0, -0.55);
    assertEquals(0.25 * MAX_LINEAR, request.vx().in(MetersPerSecond), TOLERANCE);
    assertEquals(-0.5 * MAX_ANGULAR, request.omega().in(RadiansPerSecond), TOLERANCE);
  }

  @Test
  public void testScalesToTheMaxSpeedsAndKeepsTheDirection() {
    DriveRequest full = shape(1.0, 0.0, 1.0);
    assertEquals(MAX_LINEAR, full.vx().in(MetersPerSecond), TOLERANCE);
    assertEquals(MAX_ANGULAR, full.omega().in(RadiansPerSecond), TOLERANCE);

    double component = Math.sqrt(0.5);
    DriveRequest diagonal = shape(-component, component, 0.0);
    assertEquals(-component * MAX_LINEAR, diagonal.vx().in(MetersPerSecond), TOLERANCE);
    assertEquals(component * MAX_LINEAR, diagonal.vy().in(MetersPerSecond), TOLERANCE);
  }

  @Test
  public void testDrivesFieldRelativeAndLogs() {
    JoystickDrive command = builder(sticks(1.0, 0.0, 0.0)).build();
    assertTrue(command.getRequirements().contains(drive));

    heading = Rotation2d.fromDegrees(90.0);
    ChassisVelocities velocities = run(command);
    assertEquals(0.0, velocities.vx, TOLERANCE);
    assertEquals(-MAX_LINEAR, velocities.vy, TOLERANCE);
    logger.verify(() -> Logger.recordOutput("Drive/JoystickDrive/chassisSpeeds", velocities));
  }

  @Test
  public void testDrivesInTheDriversPerspective() {
    JoystickDrive command =
        builder(sticks(1.0, 0.0, 0.0)).withDriverPerspective(() -> Rotation2d.PI).build();
    ChassisVelocities velocities = run(command);
    assertEquals(-MAX_LINEAR, velocities.vx, TOLERANCE);
    assertEquals(0.0, velocities.vy, TOLERANCE);
  }

  @Test
  public void testRunsOnlyHeldBehaviors() {
    AtomicBoolean held = new AtomicBoolean(false);
    JoystickDrive command =
        builder(sticks(1.0, 0.0, 0.0)).with(held::get, DriveBehaviors.robotRelative()).build();
    heading = Rotation2d.fromDegrees(90.0);

    assertEquals(-MAX_LINEAR, run(command).vy, TOLERANCE);
    held.set(true);
    assertEquals(MAX_LINEAR, run(command).vx, TOLERANCE);
  }

  @Test
  public void testFirstRegisteredOverrideOnAnAxisWins() {
    JoystickDrive command =
        builder(sticks(0.0, 0.0, 1.0))
            .with(() -> true, DriveBehaviors.overrideOmega(() -> RadiansPerSecond.of(1.0)))
            .with(() -> true, DriveBehaviors.overrideOmega(() -> RadiansPerSecond.of(2.0)))
            .build();
    assertEquals(1.0, run(command).omega, TOLERANCE);
  }

  @Test
  public void testScaleNeverScalesAnOverride() {
    JoystickDrive scaleFirst =
        builder(sticks(1.0, 0.0, 1.0))
            .with(() -> true, DriveBehaviors.scale(() -> 0.5))
            .with(() -> true, DriveBehaviors.overrideOmega(() -> RadiansPerSecond.of(1.0)))
            .build();
    ChassisVelocities first = run(scaleFirst);
    assertEquals(0.5 * MAX_LINEAR, first.vx, TOLERANCE);
    assertEquals(1.0, first.omega, TOLERANCE);

    JoystickDrive overrideFirst =
        builder(sticks(1.0, 0.0, 1.0))
            .with(() -> true, DriveBehaviors.overrideOmega(() -> RadiansPerSecond.of(1.0)))
            .with(() -> true, DriveBehaviors.scale(() -> 0.5))
            .build();
    ChassisVelocities second = run(overrideFirst);
    assertEquals(0.5 * MAX_LINEAR, second.vx, TOLERANCE);
    assertEquals(1.0, second.omega, TOLERANCE);
  }

  @Test
  public void testEngagesABehaviorEachTimeItsTriggerIsPressed() {
    AtomicBoolean held = new AtomicBoolean(true);
    AtomicInteger engaged = new AtomicInteger();
    DriveBehavior counting =
        new DriveBehavior() {
          @Override
          public void apply(Context context, DriveRequest request) {}

          @Override
          public void engage(Context context) {
            engaged.incrementAndGet();
          }
        };
    JoystickDrive command = builder(DriveSticks.NONE).with(held::get, counting).build();

    command.initialize();
    run(command);
    run(command);
    assertEquals(1, engaged.get());

    held.set(false);
    run(command);
    held.set(true);
    run(command);
    assertEquals(2, engaged.get());

    // Restarting the command re-engages a behavior that is still held
    command.initialize();
    run(command);
    assertEquals(3, engaged.get());
  }
}
