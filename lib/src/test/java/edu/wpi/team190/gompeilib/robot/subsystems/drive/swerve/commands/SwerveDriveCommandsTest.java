package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands;

import static edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands.DriveCommandsTestSupport.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Percent;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig.Leg;
import edu.wpi.team190.gompeilib.characterization.FeedforwardFit.Sample;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.math.CustomUnits;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDrive;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDriveConstants;
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
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.DimensionlessUnit;
import org.wpilib.units.VoltageUnit;

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
  public void testFeedforwardCharacterizationRunsTheRoutineOnTheDrive() {
    Command command =
        SwerveDriveCommands.feedforwardCharacterization(
            drive,
            CharacterizationConfig.voltage()
                .withRampRate(CustomUnits.voltsPerSecond.ofNative(1))
                .withStepOutput(Volts.of(4))
                .withQuasistaticTimeout(Seconds.of(2))
                .withDynamicTimeout(Seconds.of(1))
                .build());
    assertEquals("Characterization", command.getName());
    assertTrue(command.getRequirements().contains(drive));

    assertThrows(
        IllegalArgumentException.class,
        () ->
            SwerveDriveCommands.feedforwardCharacterization(
                drive,
                CharacterizationConfig.<DimensionlessUnit>builder()
                    .withOutputUnit(Percent)
                    .withRampRate(Percent.per(Second).ofNative(1))
                    .withStepOutput(Percent.of(10))
                    .withQuasistaticTimeout(Seconds.of(2))
                    .withDynamicTimeout(Seconds.of(1))
                    .build()));
  }

  @Test
  public void testDriveTargetDrivesEveryModuleInItsOutput() {
    SwerveDriveCommands.DriveTarget<VoltageUnit> volts =
        new SwerveDriveCommands.DriveTarget<>(drive, OutputType.VOLTAGE);
    volts.drive(Volts.of(3));
    verify(drive).runCharacterization(new OpenLoopRequest.VoltageOutput(Volts.of(3.0)));

    SwerveDriveCommands.DriveTarget<CurrentUnit> amps =
        new SwerveDriveCommands.DriveTarget<>(drive, OutputType.CURRENT);
    amps.drive(Amps.of(20));
    verify(drive).runCharacterization(new OpenLoopRequest.CurrentOutput(Amps.of(20.0)));

    assertEquals("Drive", volts.topic());
    volts.idle();
    verify(drive).stop();
    Sample sample = new Sample(1, 2, 3, 4, 5);
    when(drive.getFFCharacterizationSample(OutputType.CURRENT)).thenReturn(sample);
    assertSame(sample, amps.sample(OutputType.CURRENT));
    assertFalse(volts.atLimit(Leg.DYNAMIC_FORWARD));
    assertTrue(volts.gravity().isEmpty());
    assertTrue(volts.drumRadius().isEmpty());
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
