package edu.wpi.team190.gompeilib.robot.common.openloop;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismCore;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismTestSupport.TestMotors;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.DimensionlessUnit;
import org.wpilib.units.VoltageUnit;

public class OpenLoopControlTest extends GompeiLibTestBase {
  private static final RequestOptions OPTIONS =
      new RequestOptions(
          false, false, false, false, true, false, RequestOptions.DEFAULT_UPDATE_FREQUENCY);

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

  @Test
  public void testVoltageStartsAtZeroLimitedToTwelveVolts() {
    OpenLoopControl<VoltageUnit> control = OpenLoopControl.voltage(core, Volts.of(0.5), false);
    assertTrue(core.components().contains(control));
    assertEquals("Voltage", control.name());
    assertEquals(0.0, control.getGoal().getNewSetpoint().in(Volts), 1e-9);

    control.getGoal().setSetpoint(Volts.of(11));
    control.getGoal().increment(Volts.of(5));
    assertEquals(1.0, control.getGoal().getOffset().in(Volts), 1e-9);
  }

  @Test
  public void testVoltageSendsItsGoalAndComparesAppliedVoltage() {
    OpenLoopControl<VoltageUnit> control = OpenLoopControl.voltage(core, Volts.of(0.5), true);
    control.setGoal(Volts.of(4));
    assertTrue(core.isActive(control));
    core.periodic();
    verify(motors.motor)
        .setControl(new OpenLoopRequest.VoltageOutput(Volts.of(4), true, RequestOptions.DEFAULT));

    motors.read(t -> t.appliedVoltage = Volts.of(4.3));
    assertTrue(control.atGoal());
    assertEquals(Volts.of(4.3), control.getOutput());
    motors.read(t -> t.appliedVoltage = Volts.of(4.6));
    assertFalse(control.atGoal());
    assertTrue(control.atGoal(Volts.of(4.5)));

    assertEquals(
        new OpenLoopRequest.VoltageOutput(Volts.of(2), true, RequestOptions.DEFAULT),
        control.request(Millivolts.of(2000)));
    assertEquals(
        new OpenLoopRequest.VoltageOutput(Volts.of(2), true, OPTIONS),
        control.request(Volts.of(2), OPTIONS));
  }

  @Test
  public void testCurrentSendsTorqueCurrentAndComparesIt() {
    OpenLoopControl<CurrentUnit> control =
        OpenLoopControl.current(core, new Setpoint<>(Amps.of(0), Amps.of(1)));
    assertEquals("Current", control.name());
    control.setGoal(Amps.of(10));
    core.periodic();
    verify(motors.motor).setControl(new OpenLoopRequest.CurrentOutput(Amps.of(10)));

    motors.read(t -> t.torqueCurrent = Amps.of(10.4));
    assertTrue(control.atGoal());
    assertFalse(control.atGoal(Amps.of(12)));
    assertEquals(
        new OpenLoopRequest.CurrentOutput(Amps.of(3), 1.0, Amps.of(0), OPTIONS),
        control.request(Amps.of(3), OPTIONS));
  }

  @Test
  public void testDutyCycleStartsAtZeroLimitedToOne() {
    OpenLoopControl<DimensionlessUnit> control = OpenLoopControl.dutyCycle(core, 0.05, true);
    assertEquals("DutyCycle", control.name());
    control.getGoal().setSetpoint(Value.of(0.5));
    control.getGoal().increment(Value.of(2.0));
    assertEquals(0.5, control.getGoal().getOffset().in(Value), 1e-9);

    control.getGoal().reset();
    control.setGoal(Value.of(0.4));
    core.periodic();
    verify(motors.motor)
        .setControl(new OpenLoopRequest.DutyCycleOutput(0.4, true, RequestOptions.DEFAULT));
    motors.read(t -> t.dutyCycle = 0.43);
    assertTrue(control.atGoal());
    assertFalse(control.atGoal(Value.of(0.8)));
  }

  @Test
  public void testUsesAnExplicitTolerance() {
    OpenLoopControl<VoltageUnit> control =
        OpenLoopControl.voltage(core, new Setpoint<>(Volts.of(4), Volts.of(1)), Volts.of(1), true);
    motors.read(t -> t.appliedVoltage = Volts.of(4.8));
    assertTrue(control.atGoal());

    OpenLoopControl<CurrentUnit> current =
        OpenLoopControl.current(core, new Setpoint<>(Amps.of(5), Amps.of(1)), Amps.of(0.1));
    motors.read(t -> t.torqueCurrent = Amps.of(5.3));
    assertFalse(current.atGoal());

    OpenLoopControl<DimensionlessUnit> dutyCycle =
        OpenLoopControl.dutyCycle(
            core, new Setpoint<>(Value.of(0.5), Value.of(0.1)), Value.of(0.2), true);
    motors.read(t -> t.dutyCycle = 0.65);
    assertTrue(dutyCycle.atGoal());
  }

  @Test
  public void testReplacingTheGoalActivatesIt() {
    OpenLoopControl<VoltageUnit> control = OpenLoopControl.voltage(core, Volts.of(0.5), true);
    Setpoint<VoltageUnit> goal = new Setpoint<>(Volts.of(3), Volts.of(1));
    control.setGoal(goal);
    assertSame(goal, control.getGoal());
    assertTrue(core.isActive(control));
  }

  @Test
  public void testSelectPicksTheRequestByUnit() {
    OpenLoopControl<VoltageUnit> voltage = OpenLoopControl.voltage(core, Volts.of(0.5), true);
    OpenLoopControl<CurrentUnit> current =
        OpenLoopControl.current(core, new Setpoint<>(Amps.of(0), Amps.of(1)));

    assertEquals(
        voltage.request(Volts.of(2)),
        OpenLoopControl.select(Volts, voltage, current).apply(Volts.of(2)));
    assertEquals(
        current.request(Amps.of(15)),
        OpenLoopControl.select(Amps, voltage, current).apply(Amps.of(15)));
    assertEquals(
        voltage.request(Volts.of(2), OPTIONS),
        OpenLoopControl.select(Volts, voltage, current, OPTIONS).apply(Volts.of(2)));
    assertThrows(
        IllegalArgumentException.class, () -> OpenLoopControl.select(Percent, voltage, current));
    assertThrows(
        IllegalArgumentException.class, () -> OpenLoopControl.select(Meters, voltage, current));
  }

  @Test
  public void testLogsUnderItsName() {
    OpenLoopControl<VoltageUnit> control = OpenLoopControl.voltage(core, Volts.of(0.5), true);
    control.periodic("T");
    logger.verify(() -> Logger.recordOutputMeasure(eq("T/Voltage/Goal"), any()));
    logger.verify(() -> Logger.recordOutputMeasure(eq("T/Voltage/Offset"), any()));
    logger.verify(() -> Logger.recordOutput("T/Voltage/AtGoal", true));
  }

  @Test
  public void shouldRequireEveryArgument() {
    assertThrows(
        NullPointerException.class,
        () -> OpenLoopControl.voltage(core, (Setpoint<VoltageUnit>) null, true));
    assertThrows(
        NullPointerException.class,
        () -> OpenLoopControl.voltage(null, new Setpoint<>(Volts.of(0), Volts.of(1)), true));
    assertThrows(
        NullPointerException.class,
        () -> OpenLoopControl.current(core, new Setpoint<>(Amps.of(0), Amps.of(1)), null));
    OpenLoopControl<VoltageUnit> control = OpenLoopControl.voltage(core, Volts.of(0.5), true);
    assertThrows(NullPointerException.class, () -> control.setGoal((Setpoint<VoltageUnit>) null));
    assertThrows(
        NullPointerException.class,
        () -> control.setGoal((org.wpilib.units.Measure<VoltageUnit>) null));
    assertThrows(NullPointerException.class, () -> control.request(null));
    assertThrows(NullPointerException.class, () -> control.request(Volts.of(1), null));
  }
}
