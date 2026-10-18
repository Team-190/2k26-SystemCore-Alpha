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
import org.wpilib.units.VoltageUnit;

public class VoltageControlTest extends GompeiLibTestBase {
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
  public void testRegistersAndStartsAtZeroLimitedToTwelveVolts() {
    VoltageControl control = new VoltageControl(core, Volts.of(0.5), false);
    assertTrue(core.components().contains(control));
    assertEquals("Voltage", control.name());
    assertEquals(0.0, control.getGoal().getNewSetpoint().in(Volts), 1e-9);

    control.getGoal().setSetpoint(Volts.of(1.0));
    control.getGoal().increment(Volts.of(20));
    assertEquals(11.0, control.getGoal().getOffset().in(Volts), 1e-9);
  }

  @Test
  public void testRequestForAnyVoltageUsesTheControlsFoc() {
    VoltageControl control = new VoltageControl(core, Volts.of(0.5), true);
    assertEquals(
        new OpenLoopRequest.VoltageOutput(Volts.of(2.5), true, RequestOptions.DEFAULT),
        control.request(Millivolts.of(2500)));
    assertEquals(MechanismCore.IDLE, core.getMode());
  }

  @Test
  public void testSettingAGoalActivatesItAndSendsTheVoltage() {
    VoltageControl control = new VoltageControl(core, Volts.of(0.5), false);

    control.setGoal(Volts.of(4.0));
    assertTrue(core.isActive(control));
    core.periodic();
    verify(motors.motor)
        .setControl(
            new OpenLoopRequest.VoltageOutput(Volts.of(4.0), false, RequestOptions.DEFAULT));

    Setpoint<VoltageUnit> goal = new Setpoint<>(Volts.of(2.0), Volts.of(0.1));
    core.idle();
    control.setGoal(goal);
    assertSame(goal, control.getGoal());
    assertTrue(core.isActive(control));
  }

  @Test
  public void testAtGoalComparesTheAppliedVoltage() {
    VoltageControl control =
        new VoltageControl(core, new Setpoint<>(Volts.of(3.0), Volts.of(0.5)), true);
    motors.read(t -> t.appliedVoltage = Volts.of(2.7));

    assertTrue(control.atGoal());
    assertFalse(control.atGoal(Volts.of(2.0)));
  }

  @Test
  public void testLogsUnderTheVoltageTopic() {
    VoltageControl control = new VoltageControl(core, Volts.of(0.5), true);
    control.periodic("T");
    logger.verify(() -> Logger.recordOutputMeasure(eq("T/Voltage/Goal"), any()));
    logger.verify(() -> Logger.recordOutputMeasure(eq("T/Voltage/Offset"), any()));
    logger.verify(() -> Logger.recordOutput("T/Voltage/AtGoal", true));
  }

  @Test
  public void shouldRequireAGoal() {
    assertThrows(
        NullPointerException.class,
        () -> new VoltageControl(core, (Setpoint<VoltageUnit>) null, true));
    VoltageControl control = new VoltageControl(core, Volts.of(0.5), true);
    assertThrows(NullPointerException.class, () -> control.setGoal((Setpoint<VoltageUnit>) null));
  }
}
