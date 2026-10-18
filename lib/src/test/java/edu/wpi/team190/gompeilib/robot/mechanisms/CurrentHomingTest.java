package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMotors;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.units.DistanceUnit;

public class CurrentHomingTest extends GompeiLibTestBase {
  private MockedStatic<Logger> logger;
  private TestMotors motors;
  private MechanismCore core;
  private PositionControl<DistanceUnit> position;
  private CurrentHoming.Config<DistanceUnit> config;

  @BeforeEach
  public void setUp() {
    logger = mockStatic(Logger.class);
    motors = new TestMotors();
    core = motors.core();
    position =
        new PositionControl<>(
            core,
            MechanismConversion.drum(Meters.of(0.5)),
            new Setpoint<>(Meters.of(0), Meters.of(0.1)),
            () -> Meters.of(0.01),
            true);
    config = new CurrentHoming.Config<>(Volts.of(-1.0), Amps.of(-20), Amps.of(1), Meters.of(0.1));
  }

  @AfterEach
  public void tearDown() {
    logger.close();
  }

  @Test
  public void testDrivesIntoTheStopIgnoringSoftLimitsThenResets() {
    CurrentHoming<DistanceUnit> homing = new CurrentHoming<>(core, position, config, true);
    assertSame(config, homing.getConfig());
    position.setGoal(Meters.of(0.3));

    Command home = homing.command();
    home.initialize();
    home.execute();
    home.execute();
    assertEquals(MechanismCore.IDLE, core.getMode());
    verify(motors.motor, atLeastOnce())
        .setControl(new OpenLoopRequest.VoltageOutput(Volts.of(-1.0), true, CurrentHoming.OPTIONS));
    assertTrue(CurrentHoming.OPTIONS.ignoreSoftwareLimits());
    assertFalse(home.isFinished());

    motors.read(t -> t.torqueCurrent = Amps.of(-20.5));
    assertTrue(homing.atHardStop());
    for (int i = 0; i < 5 && !home.isFinished(); i++) {
      home.execute();
    }
    assertTrue(home.isFinished());
    verify(motors.motor)
        .setControl(new OpenLoopRequest.VoltageOutput(Volts.zero(), true, RequestOptions.DEFAULT));
    verify(motors.motor).setPosition(Radians.of(0.2));
  }

  @Test
  public void testIsNotAtTheStopAwayFromTheThreshold() {
    CurrentHoming<DistanceUnit> homing = new CurrentHoming<>(core, position, config, true);
    motors.read(t -> t.torqueCurrent = Amps.of(-5));
    assertFalse(homing.atHardStop());
  }

  @Test
  public void shouldRequireEveryArgument() {
    assertThrows(
        NullPointerException.class, () -> new CurrentHoming<>(null, position, config, true));
    assertThrows(NullPointerException.class, () -> new CurrentHoming<>(core, null, config, true));
    assertThrows(NullPointerException.class, () -> new CurrentHoming<>(core, position, null, true));
    assertThrows(
        NullPointerException.class,
        () -> new CurrentHoming.Config<>(null, Amps.of(1), Amps.of(1), Meters.of(0)));
    assertThrows(
        NullPointerException.class,
        () -> new CurrentHoming.Config<>(Volts.of(1), null, Amps.of(1), Meters.of(0)));
    assertThrows(
        NullPointerException.class,
        () -> new CurrentHoming.Config<>(Volts.of(1), Amps.of(1), null, Meters.of(0)));
    assertThrows(
        NullPointerException.class,
        () -> new CurrentHoming.Config<DistanceUnit>(Volts.of(1), Amps.of(1), Amps.of(1), null));
  }
}
