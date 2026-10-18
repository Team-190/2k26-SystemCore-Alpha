package edu.wpi.team190.gompeilib.sysid;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Subsystem;
import org.wpilib.simulation.SimHooks;
import org.wpilib.tunable.Tunable;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.DimensionlessUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.AngularVelocity;

public class KSCharacterizationTest extends GompeiLibTestBase {
  private static final double OUTPUT_TOLERANCE = 1e-6;

  private MockedStatic<Logger> logger;
  private Subsystem subsystem;
  private AtomicReference<AngularVelocity> velocity;

  @BeforeEach
  public void setUp() {
    logger = mockStatic(Logger.class);
    SimHooks.pauseTiming();
    subsystem = mock(Subsystem.class);
    velocity = new AtomicReference<>(RotationsPerSecond.of(0.0));
  }

  @AfterEach
  public void tearDown() {
    logger.close();
    SimHooks.resumeTiming();
  }

  /** Runs two half-second loops, then lets the mechanism break free and ends the command. */
  private void rampUntilMoving(KSCharacterization<?> command) {
    assertTrue(command.getRequirements().contains(subsystem));
    command.initialize();
    SimHooks.stepTiming(0.5);
    command.execute();
    SimHooks.stepTiming(0.5);
    command.execute();
    assertFalse(command.isFinished());

    velocity.set(RotationsPerSecond.of(0.1));
    assertTrue(command.isFinished());
    command.end(false);
  }

  @Test
  public void testRampsVoltageUntilTheMechanismMoves() {
    List<Measure<VoltageUnit>> outputs = new ArrayList<>();
    KSCharacterization<VoltageUnit> command =
        KSCharacterization.voltage(subsystem, outputs::add, velocity::get);
    rampUntilMoving(command);

    // Ramps at the default 0.1 V/s, then zeroes the output
    assertEquals(0.05, outputs.get(0).in(Volts), OUTPUT_TOLERANCE);
    assertEquals(0.1, outputs.get(1).in(Volts), OUTPUT_TOLERANCE);
    assertEquals(0.0, outputs.get(2).in(Volts), OUTPUT_TOLERANCE);
    logger.verify(() -> Logger.recordOutputMeasure("KS output", outputs.get(1)));
  }

  @Test
  public void testRampsCurrentUntilTheMechanismMoves() {
    List<Measure<CurrentUnit>> outputs = new ArrayList<>();
    KSCharacterization<CurrentUnit> command =
        KSCharacterization.current(subsystem, outputs::add, velocity::get);
    rampUntilMoving(command);

    // Ramps at the default 1 A/s, then zeroes the output
    assertEquals(0.5, outputs.get(0).in(Amps), OUTPUT_TOLERANCE);
    assertEquals(1.0, outputs.get(1).in(Amps), OUTPUT_TOLERANCE);
    assertEquals(0.0, outputs.get(2).in(Amps), OUTPUT_TOLERANCE);
    logger.verify(() -> Logger.recordOutputMeasure("KS output", outputs.get(1)));
  }

  @Test
  public void testRampsAnyOutputUnitAtItsRate() {
    List<Measure<DimensionlessUnit>> outputs = new ArrayList<>();
    KSCharacterization<DimensionlessUnit> command =
        new KSCharacterization<>(
            subsystem,
            Value,
            Tunable.create(Value.per(Second).ofNative(0.02)),
            outputs::add,
            velocity::get);
    rampUntilMoving(command);

    assertEquals(0.01, outputs.get(0).in(Value), OUTPUT_TOLERANCE);
    assertEquals(0.02, outputs.get(1).in(Value), OUTPUT_TOLERANCE);
    assertEquals(0.0, outputs.get(2).in(Value), OUTPUT_TOLERANCE);
  }

  @Test
  public void testReportsZeroIfEndedBeforeRamping() {
    List<Measure<VoltageUnit>> outputs = new ArrayList<>();
    KSCharacterization<VoltageUnit> command =
        KSCharacterization.voltage(subsystem, outputs::add, velocity::get);
    command.end(true);
    logger.verify(() -> Logger.recordOutputMeasure("KS output", Volts.of(0.0)));
    assertEquals(0.0, outputs.get(0).in(Volts), OUTPUT_TOLERANCE);
  }
}
