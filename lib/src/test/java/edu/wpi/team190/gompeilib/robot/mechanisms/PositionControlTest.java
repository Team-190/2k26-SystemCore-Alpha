package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.outputs.PositionRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMotors;
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
      MechanismConversion.drum(Meters.of(0.5));

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
    assertEquals("Position", control.name());

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
    PositionControl<DistanceUnit> control = control(goal);
    assertThrows(NullPointerException.class, () -> control.setGoal((Setpoint<DistanceUnit>) null));
  }
}
