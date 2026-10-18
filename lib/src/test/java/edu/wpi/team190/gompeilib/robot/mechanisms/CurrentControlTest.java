package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMotors;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.units.CurrentUnit;

public class CurrentControlTest extends GompeiLibTestBase {
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
  public void testSettingAGoalActivatesItAndSendsTheCurrent() {
    CurrentControl control = new CurrentControl(core, new Setpoint<>(Amps.of(0), Amps.of(1)));
    assertTrue(core.components().contains(control));
    assertEquals("Current", control.name());

    control.setGoal(Amps.of(10));
    assertTrue(core.isActive(control));
    core.periodic();
    verify(motors.motor).setControl(new OpenLoopRequest.CurrentOutput(Amps.of(10)));

    Setpoint<CurrentUnit> goal = new Setpoint<>(Amps.of(5), Amps.of(1));
    core.idle();
    control.setGoal(goal);
    assertSame(goal, control.getGoal());
    assertTrue(core.isActive(control));
  }

  @Test
  public void testAtGoalComparesTheTorqueCurrent() {
    CurrentControl control = new CurrentControl(core, new Setpoint<>(Amps.of(10), Amps.of(1)));
    motors.read(t -> t.torqueCurrent = Amps.of(10.3));

    assertTrue(control.atGoal());
    assertFalse(control.atGoal(Amps.of(12)));
  }

  @Test
  public void testLogsUnderTheCurrentTopic() {
    CurrentControl control = new CurrentControl(core, new Setpoint<>(Amps.of(0), Amps.of(1)));
    control.periodic("T");
    logger.verify(() -> Logger.recordOutputMeasure(eq("T/Current/Goal"), any()));
    logger.verify(() -> Logger.recordOutputMeasure(eq("T/Current/Offset"), any()));
    logger.verify(() -> Logger.recordOutput("T/Current/AtGoal", true));
  }

  @Test
  public void shouldRequireAGoal() {
    assertThrows(NullPointerException.class, () -> new CurrentControl(core, null));
    CurrentControl control = new CurrentControl(core, new Setpoint<>(Amps.of(0), Amps.of(1)));
    assertThrows(NullPointerException.class, () -> control.setGoal((Setpoint<CurrentUnit>) null));
  }
}
