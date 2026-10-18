package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.TestGains;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.io.motor.outputs.NeutralRequest;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMechanism;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMotors;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;

public class MechanismTest extends GompeiLibTestBase {
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
  public void testStopAndIdleGoThroughTheCore() {
    mechanism.setVoltageGoal(Volts.of(1));
    mechanism.stop();
    assertEquals(MechanismCore.STOP, mechanism.getMode());
    mechanism.periodic();
    verify(motors.motor).setControl(new NeutralRequest.Neutral());

    mechanism.idle();
    assertEquals(MechanismCore.IDLE, mechanism.getMode());
  }

  @Test
  public void testPassesHardwareCapabilitiesToTheMotors() {
    assertSame(motors.group, mechanism.motors());

    motors.read(t -> t.connected = true);
    assertTrue(mechanism.isConnected());

    mechanism.clearStickyFaults();
    verify(motors.motor).clearStickyFaults();

    CurrentLimits limits = new CurrentLimits(20, 30);
    mechanism.updateCurrentLimits(limits);
    verify(motors.motor).updateCurrentLimits(limits);

    SlotConfig gains = SlotConfig.position(TestGains.positionKP(2.0));
    mechanism.updateGains(gains, GainSlot.ONE);
    verify(motors.motor).updateGains(gains, GainSlot.ONE);
  }

  @Test
  public void testReportsTorqueCurrents() {
    motors.read(t -> t.torqueCurrent = Amps.of(12));
    assertEquals(Amps.of(12), mechanism.getTorqueCurrent());
    assertEquals(List.of(Amps.of(12)), mechanism.getTorqueCurrents());
  }
}
