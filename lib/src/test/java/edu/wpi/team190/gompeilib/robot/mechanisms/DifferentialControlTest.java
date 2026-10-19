package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.outputs.Differential;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.motor.outputs.PositionRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMotors;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.measure.Angle;

public class DifferentialControlTest extends GompeiLibTestBase {
  private MockedStatic<Logger> logger;
  private TestMotors motors;
  private MechanismCore core;
  private Angle currentAvg = Radians.of(0);
  private Angle currentDiff = Radians.of(0);
  private DifferentialControl<AngleUnit, AngleUnit> control;

  @BeforeEach
  public void setUp() {
    logger = mockStatic(Logger.class);
    motors = new TestMotors();
    core = motors.core();
    control =
        new DifferentialControl<>(
            core,
            MechanismTestSupport.CONVERSION,
            MechanismTestSupport.CONVERSION,
            new Setpoint<>(Radians.of(1.0), Radians.of(0.1)),
            new Setpoint<>(Radians.of(0.5), Radians.of(0.1)),
            () -> Radians.of(0.05),
            () -> Radians.of(0.05),
            () -> currentAvg,
            () -> currentDiff,
            true);
  }

  @AfterEach
  public void tearDown() {
    logger.close();
  }

  @Test
  public void testGoalsAndSetters() {
    assertEquals(OutputType.VOLTAGE, control.getOutputType());
    assertEquals("DifferentialVoltage", control.name());
    assertEquals(1.0, control.getAverageGoal().getNewSetpoint().in(Radians), 1e-9);
    assertEquals(0.5, control.getDifferenceGoal().getNewSetpoint().in(Radians), 1e-9);

    control.setAverageGoal(Radians.of(2.0));
    assertEquals(2.0, control.getAverageGoal().getNewSetpoint().in(Radians), 1e-9);

    control.setAverageGoal(Radians.of(2.5), Volts.of(1.0));
    assertEquals(OutputType.VOLTAGE, control.getOutputType());
    assertEquals(Volts.of(1.0), control.getAverageVoltageFeedforward());

    control.setAverageGoal(Radians.of(3.0), Amps.of(2.0));
    assertEquals(OutputType.CURRENT, control.getOutputType());
    assertEquals("DifferentialCurrent", control.name());
    assertEquals(Amps.of(2.0), control.getAverageCurrentFeedforward());

    control.setAverageGoal(Radians.of(3.5), 0.25);
    assertEquals(OutputType.DUTY_CYCLE, control.getOutputType());
    assertEquals("DifferentialDutyCycle", control.name());
    assertEquals(0.25, control.getAverageDutyCycleFeedforward());

    control.setDifferenceGoal(Radians.of(1.0));
    assertEquals(1.0, control.getDifferenceGoal().getNewSetpoint().in(Radians), 1e-9);

    control.setDifferenceGoal(Radians.of(1.2), Volts.of(0.5));
    assertEquals(OutputType.VOLTAGE, control.getOutputType());
    assertEquals(Volts.of(0.5), control.getDifferenceVoltageFeedforward());

    control.setDifferenceGoal(Radians.of(1.4), Amps.of(1.5));
    assertEquals(OutputType.CURRENT, control.getOutputType());
    assertEquals(Amps.of(1.5), control.getDifferenceCurrentFeedforward());

    control.setDifferenceGoal(Radians.of(1.6), 0.15);
    assertEquals(OutputType.DUTY_CYCLE, control.getOutputType());
    assertEquals(0.15, control.getDifferenceDutyCycleFeedforward());

    control.setGoals(Radians.of(4.0), Radians.of(2.0));
    assertEquals(4.0, control.getAverageGoal().getNewSetpoint().in(Radians), 1e-9);
    assertEquals(2.0, control.getDifferenceGoal().getNewSetpoint().in(Radians), 1e-9);

    Setpoint<AngleUnit> avgSp = new Setpoint<>(Radians.of(5.0), Radians.of(0.2));
    Setpoint<AngleUnit> diffSp = new Setpoint<>(Radians.of(2.5), Radians.of(0.2));
    control.setGoals(avgSp, diffSp);
    assertSame(avgSp, control.getAverageGoal());
    assertSame(diffSp, control.getDifferenceGoal());
  }

  @Test
  public void testAtGoal() {
    control.setGoals(Radians.of(1.0), Radians.of(2.0));
    currentAvg = Radians.of(0.0);
    currentDiff = Radians.of(0.0);
    assertFalse(control.atAverageGoal());
    assertFalse(control.atDifferenceGoal());
    assertFalse(control.atGoals());

    currentAvg = Radians.of(1.02);
    assertTrue(control.atAverageGoal());
    assertFalse(control.atDifferenceGoal());
    assertFalse(control.atGoals());

    currentDiff = Radians.of(2.01);
    assertTrue(control.atAverageGoal());
    assertTrue(control.atDifferenceGoal());
    assertTrue(control.atGoals());

    Command wait = control.waitUntilAtGoals();
    wait.initialize();
    assertTrue(wait.isFinished());
  }

  @Test
  public void testRequestAcrossOutputTypesAndProfileModes() {
    control.setAverageSlot(GainSlot.ZERO);
    control.setDifferenceSlot(GainSlot.ONE);
    control.setOptions(RequestOptions.DEFAULT);

    assertEquals(GainSlot.ZERO, control.getAverageSlot());
    assertEquals(GainSlot.ONE, control.getDifferenceSlot());
    assertEquals(RequestOptions.DEFAULT, control.getOptions());

    // VOLTAGE
    control.setOutputType(OutputType.VOLTAGE);
    control.setProfileMode(DifferentialControl.ProfileMode.PROFILED);
    MotorRequest req = control.request();
    assertInstanceOf(Differential.class, req);
    Differential d = (Differential) req;
    assertInstanceOf(PositionRequest.ProfiledPositionVoltage.class, d.average());
    assertInstanceOf(PositionRequest.PositionVoltage.class, d.difference());

    control.setProfileMode(DifferentialControl.ProfileMode.UNPROFILED);
    d = (Differential) control.request();
    assertInstanceOf(PositionRequest.PositionVoltage.class, d.average());
    assertInstanceOf(PositionRequest.PositionVoltage.class, d.difference());

    control.setProfileMode(DifferentialControl.ProfileMode.EXPONENTIAL);
    d = (Differential) control.request();
    assertInstanceOf(PositionRequest.ExponentialProfiledPositionVoltage.class, d.average());
    assertInstanceOf(PositionRequest.PositionVoltage.class, d.difference());

    // CURRENT
    control.setOutputType(OutputType.CURRENT);
    control.setProfileMode(DifferentialControl.ProfileMode.PROFILED);
    d = (Differential) control.request();
    assertInstanceOf(PositionRequest.ProfiledPositionCurrent.class, d.average());
    assertInstanceOf(PositionRequest.PositionCurrent.class, d.difference());

    control.setProfileMode(DifferentialControl.ProfileMode.UNPROFILED);
    d = (Differential) control.request();
    assertInstanceOf(PositionRequest.PositionCurrent.class, d.average());
    assertInstanceOf(PositionRequest.PositionCurrent.class, d.difference());

    control.setProfileMode(DifferentialControl.ProfileMode.EXPONENTIAL);
    d = (Differential) control.request();
    assertInstanceOf(PositionRequest.ExponentialProfiledPositionCurrent.class, d.average());
    assertInstanceOf(PositionRequest.PositionCurrent.class, d.difference());

    // DUTY_CYCLE
    control.setOutputType(OutputType.DUTY_CYCLE);
    control.setProfileMode(DifferentialControl.ProfileMode.PROFILED);
    d = (Differential) control.request();
    assertInstanceOf(PositionRequest.ProfiledPositionDutyCycle.class, d.average());
    assertInstanceOf(PositionRequest.PositionDutyCycle.class, d.difference());

    control.setProfileMode(DifferentialControl.ProfileMode.UNPROFILED);
    d = (Differential) control.request();
    assertInstanceOf(PositionRequest.PositionDutyCycle.class, d.average());
    assertInstanceOf(PositionRequest.PositionDutyCycle.class, d.difference());

    control.setProfileMode(DifferentialControl.ProfileMode.EXPONENTIAL);
    d = (Differential) control.request();
    assertInstanceOf(PositionRequest.ExponentialProfiledPositionDutyCycle.class, d.average());
    assertInstanceOf(PositionRequest.PositionDutyCycle.class, d.difference());
  }

  @Test
  public void testPeriodicLogging() {
    control.periodic("TestTopic");
    logger.verify(
        () ->
            Logger.recordOutputMeasure(
                "TestTopic/AverageGoal", control.getAverageGoal().getNewSetpoint()));
    logger.verify(
        () ->
            Logger.recordOutputMeasure(
                "TestTopic/DifferenceGoal", control.getDifferenceGoal().getNewSetpoint()));
    logger.verify(() -> Logger.recordOutput("TestTopic/DifferentialMode", "DifferentialVoltage"));
  }

  @Test
  public void shouldRejectNulls() {
    assertThrows(
        NullPointerException.class,
        () ->
            new DifferentialControl<>(
                core,
                null,
                MechanismTestSupport.CONVERSION,
                new Setpoint<>(Radians.of(0), Radians.of(0.1)),
                new Setpoint<>(Radians.of(0), Radians.of(0.1)),
                () -> Radians.of(0.05),
                () -> Radians.of(0.05),
                () -> currentAvg,
                () -> currentDiff,
                true));
    assertThrows(
        NullPointerException.class,
        () ->
            new DifferentialControl<>(
                core,
                MechanismTestSupport.CONVERSION,
                null,
                new Setpoint<>(Radians.of(0), Radians.of(0.1)),
                new Setpoint<>(Radians.of(0), Radians.of(0.1)),
                () -> Radians.of(0.05),
                () -> Radians.of(0.05),
                () -> currentAvg,
                () -> currentDiff,
                true));
    assertThrows(
        NullPointerException.class,
        () ->
            new DifferentialControl<>(
                core,
                MechanismTestSupport.CONVERSION,
                MechanismTestSupport.CONVERSION,
                null,
                new Setpoint<>(Radians.of(0), Radians.of(0.1)),
                () -> Radians.of(0.05),
                () -> Radians.of(0.05),
                () -> currentAvg,
                () -> currentDiff,
                true));
    assertThrows(
        NullPointerException.class,
        () ->
            new DifferentialControl<>(
                core,
                MechanismTestSupport.CONVERSION,
                MechanismTestSupport.CONVERSION,
                new Setpoint<>(Radians.of(0), Radians.of(0.1)),
                null,
                () -> Radians.of(0.05),
                () -> Radians.of(0.05),
                () -> currentAvg,
                () -> currentDiff,
                true));
    assertThrows(
        NullPointerException.class,
        () ->
            new DifferentialControl<>(
                core,
                MechanismTestSupport.CONVERSION,
                MechanismTestSupport.CONVERSION,
                new Setpoint<>(Radians.of(0), Radians.of(0.1)),
                new Setpoint<>(Radians.of(0), Radians.of(0.1)),
                null,
                () -> Radians.of(0.05),
                () -> currentAvg,
                () -> currentDiff,
                true));
    assertThrows(
        NullPointerException.class,
        () ->
            new DifferentialControl<>(
                core,
                MechanismTestSupport.CONVERSION,
                MechanismTestSupport.CONVERSION,
                new Setpoint<>(Radians.of(0), Radians.of(0.1)),
                new Setpoint<>(Radians.of(0), Radians.of(0.1)),
                () -> Radians.of(0.05),
                null,
                () -> currentAvg,
                () -> currentDiff,
                true));
    assertThrows(
        NullPointerException.class,
        () ->
            new DifferentialControl<>(
                core,
                MechanismTestSupport.CONVERSION,
                MechanismTestSupport.CONVERSION,
                new Setpoint<>(Radians.of(0), Radians.of(0.1)),
                new Setpoint<>(Radians.of(0), Radians.of(0.1)),
                () -> Radians.of(0.05),
                () -> Radians.of(0.05),
                null,
                () -> currentDiff,
                true));
    assertThrows(
        NullPointerException.class,
        () ->
            new DifferentialControl<>(
                core,
                MechanismTestSupport.CONVERSION,
                MechanismTestSupport.CONVERSION,
                new Setpoint<>(Radians.of(0), Radians.of(0.1)),
                new Setpoint<>(Radians.of(0), Radians.of(0.1)),
                () -> Radians.of(0.05),
                () -> Radians.of(0.05),
                () -> currentAvg,
                null,
                true));
    assertThrows(
        NullPointerException.class,
        () ->
            new DifferentialControl<>(
                core,
                MechanismTestSupport.CONVERSION,
                MechanismTestSupport.CONVERSION,
                new Setpoint<>(Radians.of(0), Radians.of(0.1)),
                new Setpoint<>(Radians.of(0), Radians.of(0.1)),
                () -> Radians.of(0.05),
                () -> Radians.of(0.05),
                () -> currentAvg,
                () -> currentDiff,
                true,
                null));
    assertThrows(
        NullPointerException.class, () -> control.setAverageGoal((Setpoint<AngleUnit>) null));
    assertThrows(
        NullPointerException.class,
        () -> control.setAverageGoal(Radians.of(1), (org.wpilib.units.measure.Voltage) null));
    assertThrows(
        NullPointerException.class,
        () -> control.setAverageGoal(Radians.of(1), (org.wpilib.units.measure.Current) null));
    assertThrows(
        NullPointerException.class, () -> control.setDifferenceGoal((Setpoint<AngleUnit>) null));
    assertThrows(
        NullPointerException.class,
        () -> control.setDifferenceGoal(Radians.of(1), (org.wpilib.units.measure.Voltage) null));
    assertThrows(
        NullPointerException.class,
        () -> control.setDifferenceGoal(Radians.of(1), (org.wpilib.units.measure.Current) null));
    assertThrows(
        NullPointerException.class,
        () -> control.setGoals(null, new Setpoint<>(Radians.of(0), Radians.of(0.1))));
    assertThrows(
        NullPointerException.class,
        () -> control.setGoals(new Setpoint<>(Radians.of(0), Radians.of(0.1)), null));
  }
}
