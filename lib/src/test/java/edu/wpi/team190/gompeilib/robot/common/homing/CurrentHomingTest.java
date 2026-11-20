package edu.wpi.team190.gompeilib.robot.common.homing;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismConversion;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismCore;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismTestSupport.TestMotors;
import edu.wpi.team190.gompeilib.robot.common.openloop.OpenLoopControl;
import edu.wpi.team190.gompeilib.robot.common.position.PositionControl;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.VoltageUnit;

public class CurrentHomingTest extends GompeiLibTestBase {
  private MockedStatic<Logger> logger;
  private TestMotors motors;
  private MechanismCore core;
  private PositionControl<DistanceUnit> position;
  private OpenLoopControl<VoltageUnit> voltage;
  private OpenLoopControl<CurrentUnit> current;
  private CurrentHoming.Config<DistanceUnit> config;

  @BeforeEach
  public void setUp() {
    logger = mockStatic(Logger.class);
    motors = new TestMotors();
    core = motors.core();
    position =
        new PositionControl<>(
            core,
            MechanismConversion.linear(Meters.of(0.5)),
            new Setpoint<>(Meters.of(0), Meters.of(0.1)),
            () -> Meters.of(0.01),
            true);
    voltage = OpenLoopControl.voltage(core, Volts.of(0.5), true);
    current = OpenLoopControl.current(core, new Setpoint<>(Amps.of(0), Amps.of(1)));
    config =
        new CurrentHoming.Config<>(
            CurrentHoming.Output.volts(Volts.of(-1.0)), Amps.of(-20), Amps.of(1), Meters.of(0.1));
  }

  @AfterEach
  public void tearDown() {
    logger.close();
  }

  private CurrentHoming<DistanceUnit> homing(CurrentHoming.Config<DistanceUnit> config) {
    return new CurrentHoming<>(core, position, voltage, current, config);
  }

  /** Runs one robot loop: the mechanism's periodic, then the command. */
  private void loop(Command command) {
    core.periodic();
    command.execute();
  }

  @Test
  public void testDrivesIntoTheStopIgnoringSoftLimitsThenResets() {
    CurrentHoming<DistanceUnit> homing = homing(config);
    assertTrue(core.components().contains(homing));
    assertSame(config, homing.getConfig());
    position.setGoal(Meters.of(0.3));

    Command home = homing.command();
    home.initialize();
    loop(home);
    loop(home);
    assertTrue(core.isActive(homing));
    assertEquals("Homing", core.getMode());
    assertEquals(CurrentHoming.Phase.DRIVING, homing.getPhase());
    verify(motors.motor, atLeastOnce())
        .setControl(new OpenLoopRequest.VoltageOutput(Volts.of(-1.0), true, CurrentHoming.OPTIONS));
    assertTrue(CurrentHoming.OPTIONS.ignoreSoftwareLimits());
    assertFalse(home.isFinished());

    motors.read(t -> t.torqueCurrent = Amps.of(-20.5));
    assertTrue(homing.atHardStop());
    for (int i = 0; i < 5 && !home.isFinished(); i++) {
      loop(home);
    }
    assertTrue(home.isFinished());
    home.end(false);
    verify(motors.motor).setPosition(Radians.of(0.2));

    core.periodic();
    assertEquals("Homed", core.getMode());
    verify(motors.motor, atLeastOnce())
        .setControl(new OpenLoopRequest.VoltageOutput(Volts.zero(), true, RequestOptions.DEFAULT));
  }

  @Test
  public void testHomesInTorqueCurrent() {
    CurrentHoming<DistanceUnit> homing =
        homing(
            new CurrentHoming.Config<>(
                CurrentHoming.Output.amps(Amps.of(-8.0)),
                Amps.of(-20),
                Amps.of(1),
                Meters.of(0.1)));

    Command home = homing.command();
    home.initialize();
    loop(home);
    loop(home);
    verify(motors.motor, atLeastOnce())
        .setControl(
            new OpenLoopRequest.CurrentOutput(
                Amps.of(-8.0), 1.0, Amps.zero(), CurrentHoming.OPTIONS));

    motors.read(t -> t.torqueCurrent = Amps.of(-20.5));
    for (int i = 0; i < 5 && !home.isFinished(); i++) {
      loop(home);
    }
    assertTrue(home.isFinished());
    home.end(false);
    core.periodic();
    verify(motors.motor, atLeastOnce())
        .setControl(
            new OpenLoopRequest.CurrentOutput(
                Amps.zero(), 1.0, Amps.zero(), RequestOptions.DEFAULT));
  }

  @Test
  public void testInterruptStopsDrivingAndKeepsThePosition() {
    CurrentHoming<DistanceUnit> homing = homing(config);
    Command home = homing.command();
    home.initialize();
    loop(home);
    assertEquals("Homing", core.getMode());

    home.end(true);
    assertEquals(CurrentHoming.Phase.DONE, homing.getPhase());
    assertEquals("Homed", core.getMode());
    core.periodic();
    verify(motors.motor, atLeastOnce())
        .setControl(new OpenLoopRequest.VoltageOutput(Volts.zero(), true, RequestOptions.DEFAULT));
    verify(motors.motor, never()).setPosition(any());
  }

  @Test
  public void testAnotherGoalTakesOver() {
    CurrentHoming<DistanceUnit> homing = homing(config);
    Command home = homing.command();
    home.initialize();
    loop(home);

    position.setGoal(Meters.of(0.3));
    assertTrue(core.isActive(position));
    assertFalse(core.isActive(homing));
  }

  @Test
  public void testLogsUnderTheHomingTopic() {
    CurrentHoming<DistanceUnit> homing = homing(config);
    homing.periodic("T");
    logger.verify(() -> Logger.recordOutput("T/Homing/Phase", "DONE"));
    logger.verify(() -> Logger.recordOutput("T/Homing/AtHardStop", false));
  }

  @Test
  public void testOutputCarriesItsValue() {
    assertEquals(
        new CurrentHoming.Output.Volts(Volts.of(-2)), CurrentHoming.Output.volts(Volts.of(-2)));
    assertEquals(
        new CurrentHoming.Output.Amps(Amps.of(-8)), CurrentHoming.Output.amps(Amps.of(-8)));
    assertThrows(NullPointerException.class, () -> CurrentHoming.Output.volts(null));
    assertThrows(NullPointerException.class, () -> CurrentHoming.Output.amps(null));
  }

  @Test
  public void testIsNotAtTheStopAwayFromTheThreshold() {
    CurrentHoming<DistanceUnit> homing = homing(config);
    motors.read(t -> t.torqueCurrent = Amps.of(-5));
    assertFalse(homing.atHardStop());
  }

  @Test
  public void shouldRequireEveryArgument() {
    assertThrows(
        NullPointerException.class,
        () -> new CurrentHoming<>(null, position, voltage, current, config));
    assertThrows(
        NullPointerException.class,
        () -> new CurrentHoming<>(core, null, voltage, current, config));
    assertThrows(
        NullPointerException.class,
        () -> new CurrentHoming<>(core, position, null, current, config));
    assertThrows(
        NullPointerException.class,
        () -> new CurrentHoming<>(core, position, voltage, null, config));
    assertThrows(
        NullPointerException.class,
        () -> new CurrentHoming<DistanceUnit>(core, position, voltage, current, null));
    assertThrows(
        NullPointerException.class,
        () -> new CurrentHoming.Config<DistanceUnit>(null, Amps.of(1), Amps.of(1), Meters.of(0)));
    assertThrows(
        NullPointerException.class,
        () ->
            new CurrentHoming.Config<>(
                CurrentHoming.Output.volts(Volts.of(1)), null, Amps.of(1), Meters.of(0)));
    assertThrows(
        NullPointerException.class,
        () ->
            new CurrentHoming.Config<>(
                CurrentHoming.Output.volts(Volts.of(1)), Amps.of(1), null, Meters.of(0)));
    assertThrows(
        NullPointerException.class,
        () ->
            new CurrentHoming.Config<DistanceUnit>(
                CurrentHoming.Output.volts(Volts.of(1)), Amps.of(1), Amps.of(1), null));
  }
}
