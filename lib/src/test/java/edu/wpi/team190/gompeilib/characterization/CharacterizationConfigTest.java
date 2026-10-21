package edu.wpi.team190.gompeilib.characterization;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig.Leg;
import edu.wpi.team190.gompeilib.math.CustomUnits;
import edu.wpi.team190.gompeilib.tuning.GompeiTunables;
import java.util.EnumSet;
import java.util.Set;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.VoltageUnit;

public class CharacterizationConfigTest extends GompeiLibTestBase {
  private MockTunableBackend backend;

  @Override
  protected boolean isTuningMode() {
    return true;
  }

  @BeforeEach
  public void setUp() {
    TunableRegistry.reset();
    backend = new MockTunableBackend();
    TunableRegistry.registerBackend("", backend);
  }

  @AfterEach
  public void tearDown() {
    TunableRegistry.reset();
  }

  private static CharacterizationConfig.CharacterizationConfigBuilder<VoltageUnit> voltage() {
    return CharacterizationConfig.voltage()
        .withRampRate(CustomUnits.voltsPerSecond.ofNative(0.75))
        .withStepOutput(Volts.of(4.5))
        .withQuasistaticTimeout(Seconds.of(8))
        .withDynamicTimeout(Seconds.of(2));
  }

  @Test
  public void testEveryField() {
    CharacterizationConfig<VoltageUnit> config =
        voltage()
            .withPrefix("Arm/Characterization")
            .withSettleTime(Seconds.of(0.5))
            .withLegs(Set.of(Leg.QUASISTATIC_FORWARD, Leg.DYNAMIC_FORWARD))
            .build();

    assertEquals(Volts, config.getOutputUnit());
    assertEquals(0.75, config.rampRate().get().baseUnitMagnitude(), 1e-9);
    assertEquals(4.5, config.stepOutput().get().in(Volts), 1e-9);
    assertEquals(8, config.quasistaticTimeout().get().in(Seconds), 1e-9);
    assertEquals(2, config.dynamicTimeout().get().in(Seconds), 1e-9);
    assertEquals(0.5, config.settleTime().get().in(Seconds), 1e-9);
    assertEquals(EnumSet.of(Leg.QUASISTATIC_FORWARD, Leg.DYNAMIC_FORWARD), config.getLegs());
  }

  @Test
  public void testDefaultsToEveryLegAndOneSecondSettle() {
    CharacterizationConfig<VoltageUnit> config = voltage().build();
    assertEquals(EnumSet.allOf(Leg.class), config.getLegs());
    assertEquals(CharacterizationConfig.DEFAULT_SETTLE_TIME, config.settleTime().get());
    assertEquals(1, CharacterizationConfig.DEFAULT_SETTLE_TIME.in(Seconds), 1e-9);

    assertEquals(EnumSet.allOf(Leg.class), voltage().withLegs(Set.of()).build().getLegs());
  }

  @Test
  public void testCurrentConfigUsesAmps() {
    CharacterizationConfig<CurrentUnit> config =
        CharacterizationConfig.current()
            .withRampRate(CustomUnits.ampsPerSecond.ofNative(5))
            .withStepOutput(Amps.of(40))
            .withQuasistaticTimeout(Seconds.of(10))
            .withDynamicTimeout(Seconds.of(10))
            .build();
    assertEquals(Amps, config.getOutputUnit());
    assertEquals(40, config.stepOutput().get().in(Amps), 1e-9);
  }

  @Test
  public void testTimeoutPerLeg() {
    CharacterizationConfig<VoltageUnit> config = voltage().build();
    assertEquals(Seconds.of(8), config.timeout(Leg.QUASISTATIC_FORWARD));
    assertEquals(Seconds.of(8), config.timeout(Leg.QUASISTATIC_REVERSE));
    assertEquals(Seconds.of(2), config.timeout(Leg.DYNAMIC_FORWARD));
    assertEquals(Seconds.of(2), config.timeout(Leg.DYNAMIC_REVERSE));
  }

  @Test
  public void testLegs() {
    assertArrayEquals(
        new Leg[] {
          Leg.QUASISTATIC_FORWARD, Leg.QUASISTATIC_REVERSE, Leg.DYNAMIC_FORWARD, Leg.DYNAMIC_REVERSE
        },
        Leg.values());
    assertTrue(Leg.QUASISTATIC_FORWARD.isQuasistatic());
    assertTrue(Leg.QUASISTATIC_FORWARD.isForward());
    assertTrue(Leg.QUASISTATIC_REVERSE.isQuasistatic());
    assertFalse(Leg.QUASISTATIC_REVERSE.isForward());
    assertFalse(Leg.DYNAMIC_FORWARD.isQuasistatic());
    assertTrue(Leg.DYNAMIC_FORWARD.isForward());
    assertFalse(Leg.DYNAMIC_REVERSE.isQuasistatic());
    assertFalse(Leg.DYNAMIC_REVERSE.isForward());
    assertEquals(Leg.DYNAMIC_REVERSE, Leg.valueOf("DYNAMIC_REVERSE"));
  }

  @Test
  public void testPublishesTunablesUnderThePrefix() {
    CharacterizationConfig<VoltageUnit> config = voltage().withPrefix("Published").build();
    assertNotNull(backend.getTunable("/Published/Ramp Rate"));
    assertNotNull(backend.getTunable("/Published/Step Output"));
    assertNotNull(backend.getTunable("/Published/Quasistatic Timeout"));
    assertNotNull(backend.getTunable("/Published/Dynamic Timeout"));
    assertNotNull(backend.getTunable("/Published/Settle Time"));

    backend.setDouble("/Published/Step Output", 6);
    TunableRegistry.update();
    assertEquals(6, config.stepOutput().get().in(Volts), 1e-9);
    assertEquals(
        Volts.symbol(),
        backend.getValue("/Published/Step Output" + GompeiTunables.UNIT_SUFFIX, String.class));
  }

  @Test
  public void testUnpublishedWithoutAPrefix() {
    voltage().build();
    assertThrows(IllegalArgumentException.class, () -> backend.getTunable("/Step Output"));
  }

  @Test
  public void shouldRequireTheUnitRampStepAndTimeouts() {
    assertThrows(
        NullPointerException.class,
        () -> CharacterizationConfig.<VoltageUnit>builder().withStepOutput(Volts.of(1)).build());
    assertThrows(NullPointerException.class, () -> voltage().withRampRate(null).build());
    assertThrows(NullPointerException.class, () -> voltage().withStepOutput(null).build());
    assertThrows(NullPointerException.class, () -> voltage().withQuasistaticTimeout(null).build());
    assertThrows(NullPointerException.class, () -> voltage().withDynamicTimeout(null).build());
    assertNotNull(voltage().toString());
  }
}
