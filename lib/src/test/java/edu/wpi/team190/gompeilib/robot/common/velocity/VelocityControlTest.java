package edu.wpi.team190.gompeilib.robot.common.velocity;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.outputs.Feedforward;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.motor.outputs.ProfileMode;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.motor.outputs.VelocityRequest;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismCore;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismTestSupport;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismTestSupport.TestMotors;
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

  /** A control whose current loop uses slot ONE, like a flywheel's. */
  private VelocityControl<AngularVelocityUnit> control(Setpoint<AngularVelocityUnit> goal) {
    VelocityControl<AngularVelocityUnit> control =
        new VelocityControl<>(
            core, MechanismTestSupport.CONVERSION, goal, () -> RadiansPerSecond.of(10), true);
    control.setGainSlot(OutputType.CURRENT, GainSlot.ONE);
    return control;
  }

  private static Setpoint<AngularVelocityUnit> zero() {
    return new Setpoint<>(RadiansPerSecond.of(0), RadiansPerSecond.of(5));
  }

  @Test
  public void testVoltageLoopUsesTheVoltageSlotAndFeedforward() {
    VelocityControl<AngularVelocityUnit> control = control(zero());
    assertTrue(core.components().contains(control));
    assertEquals(ProfileMode.UNPROFILED, control.getProfileMode());

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

    control.setGoal(RadiansPerSecond.of(120), Feedforward.volts(Volts.of(2.5)));
    assertEquals(Feedforward.volts(Volts.of(2.5)), control.getFeedforward());
    control.setProfileMode(ProfileMode.PROFILED);
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
  }

  @Test
  public void testCurrentLoopUsesTheCurrentSlot() {
    VelocityControl<AngularVelocityUnit> control = control(zero());

    control.setGoal(RadiansPerSecond.of(80), Feedforward.amps(Amps.of(3)));
    assertEquals(OutputType.CURRENT, control.getOutputType());
    assertEquals(GainSlot.ONE, control.getGainSlot());
    assertEquals("VelocityCurrent", control.name());
    core.periodic();
    verify(motors.motor)
        .setControl(
            new VelocityRequest.VelocityCurrent(
                RadiansPerSecond.of(80),
                RadiansPerSecondPerSecond.of(0),
                Amps.of(3),
                GainSlot.ONE,
                RequestOptions.DEFAULT));

    Setpoint<AngularVelocityUnit> goal = zero();
    control.setGoal(goal, Feedforward.amps(Amps.of(1)));
    assertSame(goal, control.getGoal());
  }

  @Test
  public void testDutyCycleLoopUsesTheVoltageSlot() {
    VelocityControl<AngularVelocityUnit> control = control(zero());

    control.setGoal(RadiansPerSecond.of(50), Feedforward.dutyCycle(0.2));
    assertEquals(OutputType.DUTY_CYCLE, control.getOutputType());
    assertEquals("VelocityDutyCycle", control.name());
    control.setProfileMode(ProfileMode.PROFILED);
    core.periodic();
    verify(motors.motor)
        .setControl(
            new VelocityRequest.ProfiledVelocityDutyCycle(
                RadiansPerSecond.of(50),
                RadiansPerSecondPerSecond.of(0),
                0.2,
                true,
                GainSlot.ZERO,
                RequestOptions.DEFAULT));
  }

  @Test
  public void testGoalWithoutFeedforwardKeepsTheOutputType() {
    VelocityControl<AngularVelocityUnit> control = control(zero());
    control.setGoal(RadiansPerSecond.of(80), Feedforward.amps(Amps.of(3)));
    control.setGoal(RadiansPerSecond.of(90));
    assertEquals(Feedforward.amps(Amps.of(3)), control.getFeedforward());

    control.setOutputType(OutputType.VOLTAGE);
    assertEquals(Feedforward.zero(OutputType.VOLTAGE), control.getFeedforward());
  }

  @Test
  public void testRejectsAnExponentialProfile() {
    VelocityControl<AngularVelocityUnit> control = control(zero());
    assertThrows(
        IllegalArgumentException.class, () -> control.setProfileMode(ProfileMode.EXPONENTIAL));
    assertEquals(ProfileMode.UNPROFILED, control.getProfileMode());
  }

  @Test
  public void testSendsItsRequestOptions() {
    VelocityControl<AngularVelocityUnit> control = control(zero());
    RequestOptions options =
        new RequestOptions(
            true, false, false, false, false, false, RequestOptions.DEFAULT_UPDATE_FREQUENCY);
    control.setOptions(options);
    assertSame(options, control.getOptions());
    control.setGoal(RadiansPerSecond.of(10));
    core.periodic();
    verify(motors.motor)
        .setControl(
            new VelocityRequest.VelocityVoltage(
                RadiansPerSecond.of(10),
                RadiansPerSecondPerSecond.of(0),
                Volts.of(0),
                true,
                GainSlot.ZERO,
                options));
  }

  @Test
  public void testStartsOnTheConstructorsOutputType() {
    VelocityControl<AngularVelocityUnit> control =
        new VelocityControl<>(
            core,
            MechanismTestSupport.CONVERSION,
            zero(),
            () -> RadiansPerSecond.of(1),
            true,
            OutputType.CURRENT);
    assertEquals(Feedforward.zero(OutputType.CURRENT), control.getFeedforward());
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
    logger.verify(() -> Logger.recordOutputMeasure("T/Velocity/Feedforward", Volts.of(0.0)));
    logger.verify(() -> Logger.recordOutput("T/Velocity/AtGoal", true));
    logger.verify(() -> Logger.recordOutput("T/Velocity/OutputType", "VOLTAGE"));

    control.setGoal(RadiansPerSecond.of(10), Feedforward.amps(Amps.of(1)));
    control.periodic("T");
    logger.verify(() -> Logger.recordOutputMeasure("T/Velocity/Feedforward", Amps.of(1)));

    control.setGoal(RadiansPerSecond.of(10), Feedforward.dutyCycle(0.5));
    control.periodic("T");
    logger.verify(() -> Logger.recordOutput("T/Velocity/Feedforward", 0.5));
  }

  @Test
  public void shouldRequireEveryArgument() {
    var conversion = MechanismTestSupport.CONVERSION;
    assertThrows(
        NullPointerException.class,
        () -> new VelocityControl<>(core, null, zero(), () -> RadiansPerSecond.of(1), true));
    assertThrows(
        NullPointerException.class,
        () -> new VelocityControl<>(core, conversion, null, () -> RadiansPerSecond.of(1), true));
    assertThrows(
        NullPointerException.class,
        () -> new VelocityControl<>(core, conversion, zero(), null, true));
    assertThrows(
        NullPointerException.class,
        () ->
            new VelocityControl<>(
                core, conversion, zero(), () -> RadiansPerSecond.of(1), true, null));

    VelocityControl<AngularVelocityUnit> control = control(zero());
    assertThrows(
        NullPointerException.class, () -> control.setGoal((Setpoint<AngularVelocityUnit>) null));
    assertThrows(
        NullPointerException.class,
        () -> control.setGoal((Setpoint<AngularVelocityUnit>) null, Feedforward.amps(Amps.of(1))));
    assertThrows(NullPointerException.class, () -> control.setGoal(RadiansPerSecond.of(1), null));
    assertThrows(NullPointerException.class, () -> control.setProfileMode(null));
    assertThrows(NullPointerException.class, () -> control.setOptions(null));
  }
}
