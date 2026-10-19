package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMechanism;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMotors;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.units.AngleUnit;

public class PositionControllableTest extends GompeiLibTestBase {
  private MockedStatic<Logger> logger;
  private TestMotors motors;
  private TestMechanism mechanism;

  @BeforeEach
  public void setUp() {
    logger = mockStatic(Logger.class);
    motors = new TestMotors();
    mechanism = new TestMechanism(motors);
  }

  @AfterEach
  public void tearDown() {
    logger.close();
  }

  @Test
  public void testDelegatesToThePositionControl() {
    mechanism.setPositionGoal(Radians.of(1));
    assertEquals("PositionVoltage", mechanism.getMode());
    assertEquals(1.0, mechanism.getPositionGoal().getNewSetpoint().in(Radians), 1e-9);

    Setpoint<AngleUnit> goal = new Setpoint<>(Radians.of(0.5), Radians.of(0.1));
    mechanism.setPositionGoal(goal);
    assertSame(goal, mechanism.getPositionGoal());

    motors.read(t -> t.position = Radians.of(0.5));
    assertTrue(mechanism.atPositionGoal());
    assertFalse(mechanism.atPositionGoal(Radians.of(2)));

    Command wait = mechanism.waitUntilAtPositionGoal();
    wait.initialize();
    assertFalse(wait.isFinished());

    mechanism.setPosition(Radians.of(0.25));
    verify(motors.motor).setPosition(Radians.of(0.25));

    mechanism.setPositionGainSlot(GainSlot.ONE);
    assertEquals(GainSlot.ONE, mechanism.getPositionGainSlot());

    mechanism.setPositionGoal(Radians.of(1.5), Volts.of(2.0));
    assertEquals(OutputType.VOLTAGE, mechanism.getPositionOutputType());
    assertEquals("PositionVoltage", mechanism.getMode());

    Setpoint<AngleUnit> vGoal = new Setpoint<>(Radians.of(1.2), Radians.of(0.1));
    mechanism.setPositionGoal(vGoal, Volts.of(1.0));
    assertSame(vGoal, mechanism.getPositionGoal());

    mechanism.setPositionGoal(Radians.of(2.0), Amps.of(3.0));
    assertEquals(OutputType.CURRENT, mechanism.getPositionOutputType());
    assertEquals("PositionCurrent", mechanism.getMode());

    Setpoint<AngleUnit> cGoal = new Setpoint<>(Radians.of(2.2), Radians.of(0.1));
    mechanism.setPositionGoal(cGoal, Amps.of(2.0));
    assertSame(cGoal, mechanism.getPositionGoal());

    mechanism.setPositionGoal(Radians.of(0.8), 0.2);
    assertEquals(OutputType.DUTY_CYCLE, mechanism.getPositionOutputType());
    assertEquals("PositionDutyCycle", mechanism.getMode());

    Setpoint<AngleUnit> dGoal = new Setpoint<>(Radians.of(0.9), Radians.of(0.1));
    mechanism.setPositionGoal(dGoal, 0.15);
    assertSame(dGoal, mechanism.getPositionGoal());

    mechanism.setPositionOutputType(OutputType.VOLTAGE);
    assertEquals(OutputType.VOLTAGE, mechanism.getPositionOutputType());
  }
}
