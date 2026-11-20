package edu.wpi.team190.gompeilib.robot.mechanisms.arm;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.encoder.inputs.AbsoluteEncoderTelemetry;
import edu.wpi.team190.gompeilib.io.encoder.interfaces.AbsoluteEncoderIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Subsystem;
import org.wpilib.units.AngleUnit;

public class AbsoluteEncoderArmTest extends GompeiLibTestBase {
  private MockedStatic<Logger> logger;
  private MotorIO motor;
  private MotorIOFactory factory;
  private Subsystem subsystem;
  private AbsoluteEncoderIO encoder;

  @BeforeEach
  public void setUp() {
    logger = mockStatic(Logger.class);
    motor = mock(MotorIO.class);
    factory = mock(MotorIOFactory.class);
    when(factory.family()).thenReturn("Mock");
    when(factory.create(any())).thenReturn(motor);
    subsystem = mock(Subsystem.class, CALLS_REAL_METHODS);
    when(subsystem.getName()).thenReturn("Test");
    encoder = mock(AbsoluteEncoderIO.class);
    doAnswer(
            invocation -> {
              ((AbsoluteEncoderTelemetry) invocation.getArgument(0)).absolutePosition =
                  Rotations.of(0.125);
              return null;
            })
        .when(encoder)
        .updateInputs(any());
  }

  @AfterEach
  public void tearDown() {
    logger.close();
  }

  @Test
  public void testReadsAndSeedsFromItsEncoder() {
    AbsoluteEncoderArm arm =
        new AbsoluteEncoderArm(factory, subsystem, "Arm", HomingArmTest.constants(), encoder);
    assertTrue(arm.core().components().contains(arm.encoderSeed()));

    arm.periodic();
    logger.verify(() -> Logger.processInputs(eq("Test/Arm/Encoder"), any()));

    arm.seedPositionFromEncoder();
    verify(motor).setPosition(Rotations.of(0.125));
  }

  @Test
  public void testKeepsTheArmsGoals() {
    Setpoint<AngleUnit> goal = new Setpoint<>(Degrees.of(10), Degrees.of(1));
    AbsoluteEncoderArm arm =
        new AbsoluteEncoderArm(factory, subsystem, "Arm", HomingArmTest.constants(), encoder, goal);
    assertSame(goal, arm.getPositionGoal());
  }

  @Test
  public void shouldRequireAnEncoder() {
    assertThrows(
        NullPointerException.class,
        () -> new AbsoluteEncoderArm(factory, subsystem, "Arm", HomingArmTest.constants(), null));
  }
}
