package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.NeutralRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMotors;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;

public class MechanismCoreTest extends GompeiLibTestBase {
  private MockedStatic<Logger> logger;
  private TestMotors motors;
  private MechanismCore core;

  private static final class FakeControl implements MechanismCore.Control {
    final MotorRequest request;
    final List<String> topics = new ArrayList<>();

    FakeControl(double volts) {
      request = new OpenLoopRequest.CurrentOutput(Amps.of(volts));
    }

    @Override
    public String name() {
      return "Fake";
    }

    @Override
    public MotorRequest request() {
      return request;
    }

    @Override
    public void periodic(String topic) {
      topics.add(topic);
    }
  }

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
  public void testTopicIsTheSubsystemAndName() {
    assertEquals("Test/Mechanism", core.getTopic());
    assertSame(motors.group, core.getMotors());
  }

  @Test
  public void testStartsIdleAndSendsNothing() {
    FakeControl control = core.add(new FakeControl(1));
    assertEquals(MechanismCore.IDLE, core.getMode());
    assertFalse(core.isActive(control));

    core.periodic();

    verify(motors.motor, never()).setControl(any());
    assertEquals(List.of("Test/Mechanism"), control.topics);
    logger.verify(() -> Logger.recordOutput("Test/Mechanism/Mode", MechanismCore.IDLE));
    logger.verify(() -> Logger.processInputs(eq("Test/Mechanism/Leader"), any()));
  }

  @Test
  public void testSendsOnlyTheActiveControlsRequest() {
    FakeControl first = core.add(new FakeControl(1));
    FakeControl second = core.add(new FakeControl(2));
    assertEquals(List.of(first, second), core.components());

    core.activate(second);
    assertTrue(core.isActive(second));
    assertEquals("Fake", core.getMode());
    core.periodic();

    verify(motors.motor).setControl(second.request);
    verify(motors.motor, never()).setControl(first.request);
    assertEquals(1, first.topics.size());
  }

  @Test
  public void testStopSendsNeutralUntilAControlIsActivated() {
    FakeControl control = core.add(new FakeControl(1));
    core.activate(control);
    core.stop();
    assertEquals(MechanismCore.STOP, core.getMode());
    core.periodic();
    verify(motors.motor).setControl(new NeutralRequest.Neutral());

    core.activate(control);
    assertEquals("Fake", core.getMode());
    core.stop();
    core.idle();
    assertEquals(MechanismCore.IDLE, core.getMode());
  }

  @Test
  public void testSetControlSendsWithoutChangingTheMode() {
    MotorRequest request = new NeutralRequest.Coast(false, Hertz.of(100));
    core.setControl(request);
    verify(motors.motor).setControl(request);
    assertEquals(MechanismCore.IDLE, core.getMode());
  }

  @Test
  public void shouldRejectAnUnregisteredControl() {
    assertThrows(IllegalArgumentException.class, () -> core.activate(new FakeControl(1)));
    assertThrows(NullPointerException.class, () -> core.add(null));
    assertThrows(
        NullPointerException.class, () -> new MechanismCore(null, motors.subsystem, "Mechanism"));
    assertThrows(
        NullPointerException.class, () -> new MechanismCore(motors.group, motors.subsystem, null));
  }
}
