package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMechanism;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMotors;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.units.VoltageUnit;

public class VoltageControllableTest extends GompeiLibTestBase {
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
  public void testDelegatesToTheVoltageControl() {
    mechanism.setVoltageGoal(Volts.of(3));
    assertEquals("Voltage", mechanism.getMode());
    assertEquals(3.0, mechanism.getVoltageGoal().getNewSetpoint().in(Volts), 1e-9);

    Setpoint<VoltageUnit> goal = new Setpoint<>(Volts.of(2), Volts.of(1));
    mechanism.setVoltageGoal(goal);
    assertSame(goal, mechanism.getVoltageGoal());

    motors.read(t -> t.appliedVoltage = Volts.of(2));
    assertTrue(mechanism.atVoltageGoal());
    assertFalse(mechanism.atVoltageGoal(Volts.of(6)));
  }
}
