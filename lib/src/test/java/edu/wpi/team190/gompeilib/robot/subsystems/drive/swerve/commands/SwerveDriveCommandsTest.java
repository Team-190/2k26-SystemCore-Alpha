package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands;

import static edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands.DriveCommandsTestSupport.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Volts;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDrive;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDriveConstants;
import edu.wpi.team190.gompeilib.sysid.KSCharacterization;
import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.ArgumentCaptor;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.simulation.SimHooks;

public class SwerveDriveCommandsTest extends GompeiLibTestBase {
  private static final double LOOP_SECONDS = 0.02;

  private MockedStatic<Logger> logger;
  private SwerveDrive drive;
  private SwerveDriveConstants constants;

  @BeforeEach
  public void setUp() {
    logger = mockStatic(Logger.class);
    drive = DriveCommandsTestSupport.drive();
    constants = DriveCommandsTestSupport.constants();
  }

  @AfterEach
  public void tearDown() {
    logger.close();
    SimHooks.resumeTiming();
  }

  @Test
  public void testJoystickDriveUsesTheDriverDeadband() {
    JoystickDrive command =
        SwerveDriveCommands.joystickDrive(
                drive,
                constants,
                new DriveSticks(() -> 0.05, () -> 0.0, () -> 0.0),
                () -> Rotation2d.ZERO)
            .build();
    command.execute();
    verify(drive).runVelocity(new ChassisVelocities(0.0, 0.0, 0.0));
  }

  @Test
  public void testRotatesInPlaceToTheTarget() {
    Command command =
        SwerveDriveCommands.rotateToAngle(
            drive, constants, () -> Rotation2d.ZERO, () -> Rotation2d.fromDegrees(90.0));
    assertTrue(command.getRequirements().contains(drive));

    command.initialize();
    command.execute();
    ArgumentCaptor<ChassisVelocities> captor = ArgumentCaptor.forClass(ChassisVelocities.class);
    verify(drive).runVelocity(captor.capture());
    assertEquals(0.0, captor.getValue().vx, TOLERANCE);
    assertEquals(0.0, captor.getValue().vy, TOLERANCE);
    assertTrue(captor.getValue().omega > 0.0);
  }

  @Test
  public void testStopHoldsTheModulesInAnX() {
    Command command = SwerveDriveCommands.stop(drive);
    assertTrue(command.getRequirements().contains(drive));
    command.execute();
    verify(drive).stopWithX();
    assertFalse(command.isFinished());
  }

  @Test
  public void testFeedforwardCharacterizationRampsVoltageForAVoltageClosedLoop() {
    Command command = SwerveDriveCommands.feedforwardCharacterization(drive, constants);
    assertInstanceOf(KSCharacterization.class, command);
    assertTrue(command.getRequirements().contains(drive));

    when(drive.getFFCharacterizationVelocity()).thenReturn(0.2);
    assertTrue(command.isFinished());
    command.end(false);
    verify(drive).runCharacterization(new OpenLoopRequest.VoltageOutput(Volts.of(0.0)));
  }

  @Test
  public void testFeedforwardCharacterizationRampsCurrentForACurrentClosedLoop() {
    SwerveDriveConstants currentConstants =
        DriveCommandsTestSupport.constants(
            DriveCommandsTestSupport.rotationGains("drive/rotation", 4.0), OutputType.CURRENT);
    Command command = SwerveDriveCommands.feedforwardCharacterization(drive, currentConstants);

    assertFalse(command.isFinished());
    command.end(false);
    verify(drive).runCharacterization(new OpenLoopRequest.CurrentOutput(Amps.of(0.0)));
  }

  @Test
  public void testWheelRadiusCharacterizationMatchesTheGyroToTheWheels() {
    double driveBaseRadius = constants.driveConfig.driveBaseRadius();
    // Over the measurement, the gyro turns 1 rad and each wheel turns 20 rad
    when(drive.getWheelRadiusCharacterizationPositions())
        .thenReturn(new double[] {0.0, 0.0, 0.0, 0.0}, new double[] {20.0, -20.0, 20.0, -20.0});
    when(drive.getRawGyroRotation())
        .thenReturn(Rotation2d.ZERO, Rotation2d.fromRadians(0.5), Rotation2d.fromRadians(1.0));

    Command command = SwerveDriveCommands.wheelRadiusCharacterization(drive, constants);
    assertTrue(command.getRequirements().contains(drive));

    SimHooks.pauseTiming();
    command.initialize();
    // Spin until the modules have oriented and measurement has started
    for (int i = 0; i < 60; i++) {
      command.execute();
      SimHooks.stepTiming(LOOP_SECONDS);
    }
    ArgumentCaptor<ChassisVelocities> captor = ArgumentCaptor.forClass(ChassisVelocities.class);
    verify(drive, atLeastOnce()).runVelocity(captor.capture());
    ChassisVelocities last = captor.getValue();
    assertEquals(0.0, last.vx, TOLERANCE);
    assertTrue(last.omega > 0.0 && last.omega <= 0.25);

    PrintStream original = System.out;
    ByteArrayOutputStream printed = new ByteArrayOutputStream();
    System.setOut(new PrintStream(printed));
    try {
      command.end(true);
    } finally {
      System.setOut(original);
    }
    double expectedRadius = (1.0 * driveBaseRadius) / 20.0;
    assertTrue(
        printed.toString().contains(String.format("%.3f meters", expectedRadius)),
        printed.toString());
  }
}
