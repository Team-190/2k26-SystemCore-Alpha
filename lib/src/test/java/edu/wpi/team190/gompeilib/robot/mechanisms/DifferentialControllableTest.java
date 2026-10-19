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
import org.wpilib.units.AngleUnit;

public class DifferentialControllableTest extends GompeiLibTestBase {
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
  public void testDelegatesToDifferentialControl() {
    mechanism.setDifferentialGoals(Radians.of(1.0), Radians.of(0.5));
    assertEquals(1.0, mechanism.getAverageGoal().getNewSetpoint().in(Radians), 1e-9);
    assertEquals(0.5, mechanism.getDifferenceGoal().getNewSetpoint().in(Radians), 1e-9);
    assertEquals("DifferentialVoltage", mechanism.getMode());

    Setpoint<AngleUnit> avgSp = new Setpoint<>(Radians.of(2.0), Radians.of(0.1));
    Setpoint<AngleUnit> diffSp = new Setpoint<>(Radians.of(1.0), Radians.of(0.1));
    mechanism.setDifferentialGoals(avgSp, diffSp);
    assertSame(avgSp, mechanism.getAverageGoal());
    assertSame(diffSp, mechanism.getDifferenceGoal());

    mechanism.setAverageGoal(Radians.of(3.0));
    assertEquals(3.0, mechanism.getAverageGoal().getNewSetpoint().in(Radians), 1e-9);

    Setpoint<AngleUnit> avgSp2 = new Setpoint<>(Radians.of(3.5), Radians.of(0.1));
    mechanism.setAverageGoal(avgSp2);
    assertSame(avgSp2, mechanism.getAverageGoal());

    mechanism.setAverageGoal(Radians.of(4.0), Volts.of(1.5));
    assertEquals(OutputType.VOLTAGE, mechanism.getDifferentialOutputType());
    assertEquals(Volts.of(1.5), mechanism.differentialControl().getAverageVoltageFeedforward());

    mechanism.setAverageGoal(Radians.of(4.5), Amps.of(2.5));
    assertEquals(OutputType.CURRENT, mechanism.getDifferentialOutputType());
    assertEquals(Amps.of(2.5), mechanism.differentialControl().getAverageCurrentFeedforward());

    mechanism.setAverageGoal(Radians.of(5.0), 0.35);
    assertEquals(OutputType.DUTY_CYCLE, mechanism.getDifferentialOutputType());
    assertEquals(0.35, mechanism.differentialControl().getAverageDutyCycleFeedforward());

    mechanism.setDifferenceGoal(Radians.of(1.5));
    assertEquals(1.5, mechanism.getDifferenceGoal().getNewSetpoint().in(Radians), 1e-9);

    Setpoint<AngleUnit> diffSp2 = new Setpoint<>(Radians.of(1.8), Radians.of(0.1));
    mechanism.setDifferenceGoal(diffSp2);
    assertSame(diffSp2, mechanism.getDifferenceGoal());

    mechanism.setDifferenceGoal(Radians.of(2.0), Volts.of(0.8));
    assertEquals(OutputType.VOLTAGE, mechanism.getDifferentialOutputType());
    assertEquals(Volts.of(0.8), mechanism.differentialControl().getDifferenceVoltageFeedforward());

    mechanism.setDifferenceGoal(Radians.of(2.2), Amps.of(1.2));
    assertEquals(OutputType.CURRENT, mechanism.getDifferentialOutputType());
    assertEquals(Amps.of(1.2), mechanism.differentialControl().getDifferenceCurrentFeedforward());

    mechanism.setDifferenceGoal(Radians.of(2.4), 0.22);
    assertEquals(OutputType.DUTY_CYCLE, mechanism.getDifferentialOutputType());
    assertEquals(0.22, mechanism.differentialControl().getDifferenceDutyCycleFeedforward());

    mechanism.setDifferentialOutputType(OutputType.CURRENT);
    assertEquals(OutputType.CURRENT, mechanism.getDifferentialOutputType());

    motors.read(t -> t.position = Radians.of(5.0));
    assertTrue(mechanism.atAverageGoal());
    assertFalse(mechanism.atDifferenceGoal());
    assertFalse(mechanism.atDifferentialGoals());

    Command wait = mechanism.waitUntilAtDifferentialGoals();
    wait.initialize();
    assertFalse(wait.isFinished());
  }
}
