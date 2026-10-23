package edu.wpi.team190.gompeilib.robot.mechanisms;

import static edu.wpi.team190.gompeilib.math.CustomUnits.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig;
import edu.wpi.team190.gompeilib.characterization.CharacterizationRoutine;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMechanism;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMotors;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.units.CurrentUnit;
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
    assertTrue(mechanism.characterizationConstraint().isEmpty());
    assertTrue(mechanism.characterizationGravity().isEmpty());
    assertTrue(mechanism.characterizationDrumRadius().isEmpty());
  }

  @Test
  public void testOpenLoopPicksTheRequestByUnit() {
    VoltageControl voltage = mechanism.voltageControl();
    CurrentControl current = mechanism.currentControl();

    assertEquals(
        voltage.request(Volts.of(2)),
        Characterizable.openLoop(Volts, voltage, current).apply(Volts.of(2)));
    assertEquals(
        current.request(Amps.of(15)),
        Characterizable.openLoop(Amps, voltage, current).apply(Amps.of(15)));
    assertThrows(
        IllegalArgumentException.class, () -> Characterizable.openLoop(Percent, voltage, current));
    assertThrows(
        IllegalArgumentException.class, () -> Characterizable.openLoop(Meters, voltage, current));
  }

  @Test
  public void testCharacterizesInTorqueCurrent() {
    CharacterizationConfig<CurrentUnit> config =
        CharacterizationConfig.current()
            .withRampRate(AmpsPerSecond.ofNative(2))
            .withStepOutput(Amps.of(20))
            .withQuasistaticTimeout(Seconds.of(3))
            .withDynamicTimeout(Seconds.of(3))
            .build();

    CharacterizationRoutine.MechanismTarget<CurrentUnit> target =
        (CharacterizationRoutine.MechanismTarget<CurrentUnit>)
            mechanism.characterizationTarget(
                config, mechanism.voltageControl(), mechanism.currentControl());
    assertEquals(
        mechanism.currentControl().request(Amps.of(5)), target.request().apply(Amps.of(5)));

    Command command =
        mechanism.characterize(config, mechanism.voltageControl(), mechanism.currentControl());
    command.initialize();
    command.execute();
    command.execute();
    verify(motors.motor, atLeastOnce()).setControl(isA(OpenLoopRequest.CurrentOutput.class));
  }

  @Test
  public void testTargetDrivesThroughTheCore() {
    CharacterizationRoutine.MechanismTarget<VoltageUnit> target =
        (CharacterizationRoutine.MechanismTarget<VoltageUnit>)
            mechanism.<VoltageUnit>characterizationTarget(mechanism.voltageControl()::request);
    assertSame(mechanism.core(), target.core());
    assertTrue(target.constraint().isEmpty());
    assertTrue(target.gravity().isEmpty());
    assertTrue(target.drumRadius().isEmpty());
    assertEquals(
        new OpenLoopRequest.VoltageOutput(Volts.of(1.5), true, RequestOptions.DEFAULT),
        target.request().apply(Volts.of(1.5)));
  }

  @Test
  public void testCharacterizeIdlesAndDrivesTheMechanism() {
    mechanism.setVoltageGoal(Volts.of(4));
    Command command =
        mechanism.characterize(
            CharacterizationConfig.voltage()
                .withRampRate(VoltsPerSecond.ofNative(1))
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
        () -> Logger.recordOutput("Test/Mechanism/Characterization/State", "quasistatic-forward"),
        atLeastOnce());
  }
}
