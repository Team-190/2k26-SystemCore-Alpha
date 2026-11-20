package edu.wpi.team190.gompeilib.robot.common.position;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.outputs.Feedforward;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.motor.outputs.PositionRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.ProfileMode;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismConversion;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismCore;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismTestSupport.TestMotors;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.measure.Distance;

public class PositionControlTest extends GompeiLibTestBase {
  private MockedStatic<Logger> logger;
  private TestMotors motors;
  private MechanismCore core;
  private final MechanismConversion<DistanceUnit, ?, ?> drum =
      MechanismConversion.linear(Meters.of(0.5));

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

  private PositionControl<DistanceUnit> control(Setpoint<DistanceUnit> goal) {
    return new PositionControl<>(core, drum, goal, () -> Meters.of(0.01), false);
  }

  @Test
  public void testSendsTheProfiledGoalThroughTheConversion() {
    PositionControl<DistanceUnit> control = control(new Setpoint<>(Meters.of(0), Meters.of(0.1)));
    assertTrue(core.components().contains(control));
    assertEquals("PositionVoltage", control.name());
    assertEquals(Feedforward.zero(OutputType.VOLTAGE), control.getFeedforward());

    control.setGoal(Meters.of(1.0));
    control.setGainSlot(GainSlot.TWO);
    assertTrue(core.isActive(control));
    core.periodic();
    verify(motors.motor)
        .setControl(
            new PositionRequest.ProfiledPositionVoltage(
                Radians.of(2.0), Volts.of(0.0), false, GainSlot.TWO, RequestOptions.DEFAULT));
    assertEquals(GainSlot.TWO, control.getGainSlot());

    Setpoint<DistanceUnit> goal = new Setpoint<>(Meters.of(0.3), Meters.of(0.1));
    core.idle();
    control.setGoal(goal);
    assertSame(goal, control.getGoal());
    assertTrue(core.isActive(control));
  }

  @Test
  public void testFeedforwardPicksTheOutputType() {
    PositionControl<DistanceUnit> control = control(new Setpoint<>(Meters.of(0), Meters.of(0.1)));

    control.setGoal(Meters.of(1.0), Feedforward.amps(Amps.of(5.0)));
    assertEquals(OutputType.CURRENT, control.getOutputType());
    assertEquals(Feedforward.amps(Amps.of(5.0)), control.getFeedforward());
    assertEquals("PositionCurrent", control.name());
    core.periodic();
    verify(motors.motor)
        .setControl(
            new PositionRequest.ProfiledPositionCurrent(
                Radians.of(2.0), Amps.of(5.0), GainSlot.ZERO, RequestOptions.DEFAULT));

    control.setGoal(Meters.of(1.0), Feedforward.dutyCycle(0.25));
    assertEquals(OutputType.DUTY_CYCLE, control.getOutputType());
    assertEquals("PositionDutyCycle", control.name());
    core.periodic();
    verify(motors.motor)
        .setControl(
            new PositionRequest.ProfiledPositionDutyCycle(
                Radians.of(2.0), 0.25, false, GainSlot.ZERO, RequestOptions.DEFAULT));

    Setpoint<DistanceUnit> sp = new Setpoint<>(Meters.of(0.5), Meters.of(0.1));
    control.setGoal(sp, Feedforward.volts(Volts.of(1.5)));
    assertSame(sp, control.getGoal());
    assertEquals(OutputType.VOLTAGE, control.getOutputType());
    assertEquals(Feedforward.volts(Volts.of(1.5)), control.getFeedforward());
  }

  @Test
  public void testGoalWithoutFeedforwardKeepsTheFeedforward() {
    PositionControl<DistanceUnit> control = control(new Setpoint<>(Meters.of(0), Meters.of(0.1)));
    control.setGoal(Meters.of(1.0), Feedforward.amps(Amps.of(5.0)));
    control.setGoal(Meters.of(0.5));
    assertEquals(Feedforward.amps(Amps.of(5.0)), control.getFeedforward());
  }

  @Test
  public void testSettingTheOutputTypeZeroesTheFeedforward() {
    PositionControl<DistanceUnit> control = control(new Setpoint<>(Meters.of(0), Meters.of(0.1)));
    control.setGoal(Meters.of(1.0), Feedforward.volts(Volts.of(2.0)));
    control.setOutputType(OutputType.CURRENT);
    assertEquals(Feedforward.zero(OutputType.CURRENT), control.getFeedforward());
    assertEquals(OutputType.CURRENT, control.getOutputType());
  }

  @Test
  public void testStartsOnTheConstructorsOutputType() {
    PositionControl<DistanceUnit> control =
        new PositionControl<>(
            core,
            drum,
            new Setpoint<>(Meters.of(0), Meters.of(0.1)),
            () -> Meters.of(0.01),
            false,
            OutputType.CURRENT);
    assertEquals(Feedforward.zero(OutputType.CURRENT), control.getFeedforward());
  }

  @Test
  public void testProfileModes() {
    PositionControl<DistanceUnit> control = control(new Setpoint<>(Meters.of(0), Meters.of(0.1)));
    control.setGoal(Meters.of(1.0), Feedforward.volts(Volts.of(2.0)));
    assertEquals(ProfileMode.PROFILED, control.getProfileMode());

    control.setProfileMode(ProfileMode.UNPROFILED);
    core.periodic();
    verify(motors.motor)
        .setControl(
            new PositionRequest.PositionVoltage(
                Radians.of(2.0),
                RadiansPerSecond.of(0.0),
                Volts.of(2.0),
                false,
                GainSlot.ZERO,
                RequestOptions.DEFAULT));

    control.setProfileMode(ProfileMode.EXPONENTIAL);
    core.periodic();
    verify(motors.motor)
        .setControl(
            new PositionRequest.ExponentialProfiledPositionVoltage(
                Radians.of(2.0), Volts.of(2.0), false, GainSlot.ZERO, RequestOptions.DEFAULT));
  }

  @Test
  public void testGainSlotPerOutputType() {
    PositionControl<DistanceUnit> control = control(new Setpoint<>(Meters.of(0), Meters.of(0.1)));
    control.setGainSlot(OutputType.CURRENT, GainSlot.ONE);
    assertEquals(GainSlot.ZERO, control.getGainSlot());

    control.setGoal(Meters.of(1.0), Feedforward.amps(Amps.of(0.0)));
    assertEquals(GainSlot.ONE, control.getGainSlot());
    core.periodic();
    verify(motors.motor)
        .setControl(
            new PositionRequest.ProfiledPositionCurrent(
                Radians.of(2.0), Amps.of(0.0), GainSlot.ONE, RequestOptions.DEFAULT));
  }

  @Test
  public void testSendsItsRequestOptions() {
    PositionControl<DistanceUnit> control = control(new Setpoint<>(Meters.of(0), Meters.of(0.1)));
    RequestOptions options =
        new RequestOptions(
            false, false, false, false, true, false, RequestOptions.DEFAULT_UPDATE_FREQUENCY);
    control.setOptions(options);
    assertSame(options, control.getOptions());
    control.setGoal(Meters.of(1.0));
    core.periodic();
    verify(motors.motor)
        .setControl(
            new PositionRequest.ProfiledPositionVoltage(
                Radians.of(2.0), Volts.of(0.0), false, GainSlot.ZERO, options));
  }

  @Test
  public void testReadsAndResetsPositionInMechanismUnits() {
    PositionControl<DistanceUnit> control = control(new Setpoint<>(Meters.of(0.5), Meters.of(0.1)));
    motors.read(t -> t.position = Radians.of(0.99));

    assertEquals(0.495, ((Distance) control.getPosition()).in(Meters), 1e-9);
    assertTrue(control.atGoal());
    assertFalse(control.atGoal(Meters.of(0.6)));

    control.setPosition(Meters.of(0.25));
    verify(motors.motor).setPosition(Radians.of(0.5));
  }

  @Test
  public void testWaitsALoopBeforeCheckingTheGoal() {
    PositionControl<DistanceUnit> control = control(new Setpoint<>(Meters.of(0), Meters.of(0.1)));
    motors.read(t -> t.position = Radians.of(0));
    assertTrue(control.atGoal());

    Command wait = control.waitUntilAtGoal();
    wait.initialize();
    assertFalse(wait.isFinished());
  }

  @Test
  public void testLogsUnderThePositionTopic() {
    PositionControl<DistanceUnit> control = control(new Setpoint<>(Meters.of(0), Meters.of(0.1)));
    control.periodic("T");
    logger.verify(() -> Logger.recordOutputMeasure(eq("T/Position/Goal"), any()));
    logger.verify(() -> Logger.recordOutputMeasure(eq("T/Position/Offset"), any()));
    logger.verify(() -> Logger.recordOutputMeasure(eq("T/Position/Measured"), any()));
    logger.verify(() -> Logger.recordOutput("T/Position/AtGoal", true));
    logger.verify(() -> Logger.recordOutput("T/Position/GainSlot", "ZERO"));
    logger.verify(() -> Logger.recordOutput("T/Position/OutputType", "VOLTAGE"));
  }

  @Test
  public void shouldRequireEveryArgument() {
    Setpoint<DistanceUnit> goal = new Setpoint<>(Meters.of(0), Meters.of(0.1));
    assertThrows(
        NullPointerException.class,
        () -> new PositionControl<>(core, null, goal, () -> Meters.of(0.01), true));
    assertThrows(
        NullPointerException.class,
        () -> new PositionControl<>(core, drum, null, () -> Meters.of(0.01), true));
    assertThrows(
        NullPointerException.class, () -> new PositionControl<>(core, drum, goal, null, true));
    assertThrows(
        NullPointerException.class,
        () -> new PositionControl<>(core, drum, goal, () -> Meters.of(0.01), true, null));
    PositionControl<DistanceUnit> control = control(goal);
    assertThrows(NullPointerException.class, () -> control.setGoal((Setpoint<DistanceUnit>) null));
    assertThrows(NullPointerException.class, () -> control.setGoal(Meters.of(1.0), null));
    assertThrows(
        NullPointerException.class,
        () -> control.setGoal((Setpoint<DistanceUnit>) null, Feedforward.volts(Volts.of(1))));
    assertThrows(NullPointerException.class, () -> control.setProfileMode(null));
    assertThrows(NullPointerException.class, () -> control.setOptions(null));
    assertThrows(NullPointerException.class, () -> control.setGainSlot(null));
  }
}
