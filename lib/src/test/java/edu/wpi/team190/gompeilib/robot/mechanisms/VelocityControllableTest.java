package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMechanism;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMotors;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.units.AngularVelocityUnit;

public class VelocityControllableTest extends GompeiLibTestBase {
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
  public void testDelegatesToTheVelocityControl() {
    mechanism.setVelocityGoal(RadiansPerSecond.of(50));
    assertEquals("VelocityVoltage", mechanism.getMode());
    assertEquals(50.0, mechanism.getVelocityGoal().getNewSetpoint().in(RadiansPerSecond), 1e-9);

    Setpoint<AngularVelocityUnit> goal =
        new Setpoint<>(RadiansPerSecond.of(20), RadiansPerSecond.of(1));
    mechanism.setVelocityGoal(goal);
    assertSame(goal, mechanism.getVelocityGoal());

    mechanism.setVelocityGoal(RadiansPerSecond.of(30), Amps.of(2));
    assertEquals("VelocityCurrent", mechanism.getMode());
    assertEquals(Amps.of(2), mechanism.getVelocityFeedforward());

    Setpoint<AngularVelocityUnit> other =
        new Setpoint<>(RadiansPerSecond.of(10), RadiansPerSecond.of(1));
    mechanism.setVelocityGoal(other, Amps.of(1));
    assertSame(other, mechanism.getVelocityGoal());

    mechanism.setVelocityGoal(RadiansPerSecond.of(35), Volts.of(1.5));
    assertEquals("VelocityVoltage", mechanism.getMode());
    assertEquals(Volts.of(1.5), mechanism.getVelocityVoltageFeedforward());

    Setpoint<AngularVelocityUnit> vGoal =
        new Setpoint<>(RadiansPerSecond.of(15), RadiansPerSecond.of(1));
    mechanism.setVelocityGoal(vGoal, Volts.of(2.0));
    assertSame(vGoal, mechanism.getVelocityGoal());

    mechanism.setVelocityGoal(RadiansPerSecond.of(40), 0.3);
    assertEquals("VelocityDutyCycle", mechanism.getMode());
    assertEquals(0.3, mechanism.getVelocityDutyCycleFeedforward());

    Setpoint<AngularVelocityUnit> dGoal =
        new Setpoint<>(RadiansPerSecond.of(25), RadiansPerSecond.of(1));
    mechanism.setVelocityGoal(dGoal, 0.25);
    assertSame(dGoal, mechanism.getVelocityGoal());

    mechanism.setVelocityOutputType(OutputType.CURRENT);
    assertEquals(OutputType.CURRENT, mechanism.getVelocityOutputType());

    motors.read(t -> t.velocity = RadiansPerSecond.of(25));
    assertTrue(mechanism.atVelocityGoal());
    assertFalse(mechanism.atVelocityGoal(RadiansPerSecond.of(40)));

    Command wait = mechanism.waitUntilAtVelocityGoal();
    wait.initialize();
    assertFalse(wait.isFinished());
  }
}
