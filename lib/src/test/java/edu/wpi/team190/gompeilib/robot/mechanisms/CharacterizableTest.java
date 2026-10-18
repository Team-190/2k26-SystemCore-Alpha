package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.math.CustomUnits;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMechanism;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMotors;
import edu.wpi.team190.gompeilib.sysid.CustomSysIdRoutine;
import edu.wpi.team190.gompeilib.sysid.SysIdConfig;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.units.VoltageUnit;

public class CharacterizableTest extends GompeiLibTestBase {
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
  public void testIsUnconstrainedByDefault() {
    assertTrue(mechanism.sysIdConstraint().isEmpty());
  }

  @Test
  public void testTargetDrivesThroughTheCore() {
    CustomSysIdRoutine.Target<VoltageUnit> target =
        mechanism.sysIdTarget(mechanism.voltageControl()::request);
    assertSame(mechanism.core(), target.core());
    assertTrue(target.constraint().isEmpty());
    assertEquals(
        new OpenLoopRequest.VoltageOutput(Volts.of(1.5), true, RequestOptions.DEFAULT),
        target.request().apply(Volts.of(1.5)));
  }

  @Test
  public void testCharacterizeIdlesAndDrivesTheMechanism() {
    mechanism.setVoltageGoal(Volts.of(4));
    Command command =
        mechanism.characterize(
            SysIdConfig.voltage()
                .withRampRate(CustomUnits.voltsPerSecond.ofNative(1))
                .withStepOutput(Volts.of(3))
                .withQuasistaticTimeout(Seconds.of(3))
                .withDynamicTimeout(Seconds.of(3))
                .build(),
            mechanism.voltageControl()::request);
    assertTrue(command.getRequirements().contains(motors.subsystem));

    command.initialize();
    command.execute();
    command.execute();
    assertEquals(MechanismCore.IDLE, mechanism.getMode());
    verify(motors.motor, atLeastOnce()).setControl(isA(OpenLoopRequest.VoltageOutput.class));
    logger.verify(
        () -> Logger.recordOutput("Test/Mechanism/SysId/State", "quasistatic-forward"),
        atLeastOnce());
  }
}
