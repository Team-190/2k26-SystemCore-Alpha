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
  public void testVoltageLoopUsesTheVoltageSlotAndFeedforward() {
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

    control.setGoal(RadiansPerSecond.of(120), Volts.of(2.5));
    assertEquals(Volts.of(2.5), control.getVoltageFeedforward());
    core.periodic();
    verify(motors.motor)
        .setControl(
            new VelocityRequest.VelocityVoltage(
                RadiansPerSecond.of(120),
                RadiansPerSecondPerSecond.of(0),
                Volts.of(2.5),
                true,
                GainSlot.ZERO,
                RequestOptions.DEFAULT));

    control.setProfiled(true);
    core.periodic();
    verify(motors.motor)
        .setControl(
            new VelocityRequest.ProfiledVelocityVoltage(
                RadiansPerSecond.of(120),
                RadiansPerSecondPerSecond.of(0),
                Volts.of(2.5),
                true,
                GainSlot.ZERO,
                RequestOptions.DEFAULT));

    Setpoint<AngularVelocityUnit> goal = zero();
    control.setGoal(goal, Volts.of(1.0));
    assertSame(goal, control.getGoal());
    assertEquals(Volts.of(1.0), control.getVoltageFeedforward());
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

    control.setProfiled(true);
    core.periodic();
    verify(motors.motor)
        .setControl(
            new VelocityRequest.ProfiledVelocityCurrent(
                RadiansPerSecond.of(200),
                RadiansPerSecondPerSecond.of(0),
                Amps.of(3),
                GainSlot.ONE,
                RequestOptions.DEFAULT));

    // When setting goal without feedforward, output type is preserved
    control.setGoal(RadiansPerSecond.of(250));
    assertEquals(OutputType.CURRENT, control.getOutputType());

    Setpoint<AngularVelocityUnit> goal = zero();
    core.idle();
    control.setGoal(goal, Amps.of(1));
    assertSame(goal, control.getGoal());
    assertTrue(core.isActive(control));
    assertEquals(Amps.of(1), control.getFeedforward());

    control.setGoal(goal);
    assertSame(goal, control.getGoal());
  }

  @Test
  public void testDutyCycleLoopUsesDutyCycleSlotAndFeedforward() {
    VelocityControl<AngularVelocityUnit> control =
        new VelocityControl<>(
            core,
            MechanismTestSupport.CONVERSION,
            zero(),
            () -> RadiansPerSecond.of(10),
            true,
            OutputType.DUTY_CYCLE,
            GainSlot.TWO);
    assertEquals(OutputType.DUTY_CYCLE, control.getOutputType());

    control.setGoal(RadiansPerSecond.of(150), 0.2);
    assertEquals("VelocityDutyCycle", control.name());
    assertEquals(0.2, control.getDutyCycleFeedforward());
    core.periodic();
    verify(motors.motor)
        .setControl(
            new VelocityRequest.VelocityDutyCycle(
                RadiansPerSecond.of(150),
                RadiansPerSecondPerSecond.of(0),
                0.2,
                true,
                GainSlot.TWO,
                RequestOptions.DEFAULT));

    control.setProfiled(true);
    core.periodic();
    verify(motors.motor)
        .setControl(
            new VelocityRequest.ProfiledVelocityDutyCycle(
                RadiansPerSecond.of(150),
                RadiansPerSecondPerSecond.of(0),
                0.2,
                true,
                GainSlot.TWO,
                RequestOptions.DEFAULT));

    Setpoint<AngularVelocityUnit> goal = zero();
    control.setGoal(goal, 0.1);
    assertSame(goal, control.getGoal());
    assertEquals(0.1, control.getDutyCycleFeedforward());
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
    logger.verify(() -> Logger.recordOutput("T/Velocity/OutputType", "VOLTAGE"));

    control.setGoal(RadiansPerSecond.of(10), Amps.of(1));
    control.periodic("T");

    control.setGoal(RadiansPerSecond.of(10), 0.5);
    control.periodic("T");
    logger.verify(() -> Logger.recordOutput("T/Velocity/Feedforward", 0.5));
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
                core,
                conversion,
                zero(),
                () -> RadiansPerSecond.of(1),
                true,
                (GainSlot) null,
                GainSlot.ONE));
    assertThrows(
        NullPointerException.class,
        () ->
            new VelocityControl<>(
                core, conversion, zero(), () -> RadiansPerSecond.of(1), true, GainSlot.ZERO, null));
    assertThrows(
        NullPointerException.class,
        () ->
            new VelocityControl<>(
                core,
                conversion,
                zero(),
                () -> RadiansPerSecond.of(1),
                true,
                null,
                GainSlot.ZERO,
                GainSlot.ONE));

    VelocityControl<AngularVelocityUnit> control = control(zero());
    assertThrows(
        NullPointerException.class, () -> control.setGoal((Setpoint<AngularVelocityUnit>) null));
    assertThrows(
        NullPointerException.class,
        () -> control.setGoal((Setpoint<AngularVelocityUnit>) null, Amps.of(1)));
    assertThrows(
        NullPointerException.class,
        () -> control.setGoal(RadiansPerSecond.of(1), (org.wpilib.units.measure.Current) null));
    assertThrows(
        NullPointerException.class,
        () -> control.setGoal((Setpoint<AngularVelocityUnit>) null, Volts.of(1)));
    assertThrows(
        NullPointerException.class,
        () -> control.setGoal(RadiansPerSecond.of(1), (org.wpilib.units.measure.Voltage) null));
    assertThrows(
        NullPointerException.class,
        () -> control.setGoal((Setpoint<AngularVelocityUnit>) null, 0.5));
  }
}
