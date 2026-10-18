package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.motor.outputs.VelocityRequest;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMotors;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.units.AngularVelocityUnit;

public class VelocityControlTest extends GompeiLibTestBase {
  private MockedStatic<Logger> logger;
  private TestMotors motors;
  private MechanismCore core;

  @BeforeEach
  public void setUp() {
    logger = mockStatic(Logger.class);
    motors = new TestMotors();
    core = motors.core();
  }

  @AfterEach
  public void tearDown() {
    logger.close();
  }

  private VelocityControl<AngularVelocityUnit> control(Setpoint<AngularVelocityUnit> goal) {
    return new VelocityControl<>(
        core,
        MechanismTestSupport.CONVERSION,
        goal,
        () -> RadiansPerSecond.of(10),
        true,
        GainSlot.ZERO,
        GainSlot.ONE);
  }

  private static Setpoint<AngularVelocityUnit> zero() {
    return new Setpoint<>(RadiansPerSecond.of(0), RadiansPerSecond.of(5));
  }

  @Test
  public void testVoltageLoopUsesTheVoltageSlot() {
    VelocityControl<AngularVelocityUnit> control = control(zero());
    assertTrue(core.components().contains(control));

    control.setGoal(RadiansPerSecond.of(100));
    assertTrue(core.isActive(control));
    assertEquals(OutputType.VOLTAGE, control.getOutputType());
    assertEquals("VelocityVoltage", control.name());
    core.periodic();
    verify(motors.motor)
        .setControl(
            new VelocityRequest.VelocityVoltage(
                RadiansPerSecond.of(100),
                RadiansPerSecondPerSecond.of(0),
                Volts.of(0),
                true,
                GainSlot.ZERO,
                RequestOptions.DEFAULT));

    Setpoint<AngularVelocityUnit> goal = zero();
    control.setGoal(goal);
    assertSame(goal, control.getGoal());
  }

  @Test
  public void testCurrentLoopUsesTheCurrentSlotAndFeedforward() {
    VelocityControl<AngularVelocityUnit> control = control(zero());

    control.setGoal(RadiansPerSecond.of(200), Amps.of(3));
    assertEquals(OutputType.CURRENT, control.getOutputType());
    assertEquals("VelocityCurrent", control.name());
    assertEquals(Amps.of(3), control.getFeedforward());
    core.periodic();
    verify(motors.motor)
        .setControl(
            new VelocityRequest.VelocityCurrent(
                RadiansPerSecond.of(200),
                RadiansPerSecondPerSecond.of(0),
                Amps.of(3),
                GainSlot.ONE,
                RequestOptions.DEFAULT));

    Setpoint<AngularVelocityUnit> goal = zero();
    core.idle();
    control.setGoal(goal, Amps.of(1));
    assertSame(goal, control.getGoal());
    assertTrue(core.isActive(control));
    assertEquals(Amps.of(1), control.getFeedforward());
  }

  @Test
  public void testReadsAndComparesVelocity() {
    VelocityControl<AngularVelocityUnit> control =
        control(new Setpoint<>(RadiansPerSecond.of(100), RadiansPerSecond.of(5)));
    motors.read(t -> t.velocity = RadiansPerSecond.of(95));

    assertEquals(95.0, control.getVelocity().in(RadiansPerSecond), 1e-9);
    assertTrue(control.atGoal());
    assertFalse(control.atGoal(RadiansPerSecond.of(120)));

    Command wait = control.waitUntilAtGoal();
    wait.initialize();
    assertFalse(wait.isFinished());
  }

  @Test
  public void testLogsUnderTheVelocityTopic() {
    VelocityControl<AngularVelocityUnit> control = control(zero());
    control.periodic("T");
    logger.verify(() -> Logger.recordOutputMeasure(eq("T/Velocity/Goal"), any()));
    logger.verify(() -> Logger.recordOutputMeasure(eq("T/Velocity/Offset"), any()));
    logger.verify(() -> Logger.recordOutputMeasure(eq("T/Velocity/Measured"), any()));
    logger.verify(() -> Logger.recordOutputMeasure(eq("T/Velocity/Feedforward"), any()));
    logger.verify(() -> Logger.recordOutput("T/Velocity/AtGoal", true));
  }

  @Test
  public void shouldRequireEveryArgument() {
    var conversion = MechanismTestSupport.CONVERSION;
    assertThrows(
        NullPointerException.class,
        () ->
            new VelocityControl<>(
                core,
                null,
                zero(),
                () -> RadiansPerSecond.of(1),
                true,
                GainSlot.ZERO,
                GainSlot.ONE));
    assertThrows(
        NullPointerException.class,
        () ->
            new VelocityControl<>(
                core,
                conversion,
                null,
                () -> RadiansPerSecond.of(1),
                true,
                GainSlot.ZERO,
                GainSlot.ONE));
    assertThrows(
        NullPointerException.class,
        () ->
            new VelocityControl<>(
                core, conversion, zero(), null, true, GainSlot.ZERO, GainSlot.ONE));
    assertThrows(
        NullPointerException.class,
        () ->
            new VelocityControl<>(
                core, conversion, zero(), () -> RadiansPerSecond.of(1), true, null, GainSlot.ONE));
    assertThrows(
        NullPointerException.class,
        () ->
            new VelocityControl<>(
                core, conversion, zero(), () -> RadiansPerSecond.of(1), true, GainSlot.ZERO, null));
    VelocityControl<AngularVelocityUnit> control = control(zero());
    assertThrows(
        NullPointerException.class, () -> control.setGoal((Setpoint<AngularVelocityUnit>) null));
    assertThrows(
        NullPointerException.class,
        () -> control.setGoal((Setpoint<AngularVelocityUnit>) null, Amps.of(1)));
    assertThrows(NullPointerException.class, () -> control.setGoal(RadiansPerSecond.of(1), null));
  }
}
