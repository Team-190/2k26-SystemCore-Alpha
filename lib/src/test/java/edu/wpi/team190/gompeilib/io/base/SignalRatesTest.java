package edu.wpi.team190.gompeilib.io.base;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Hertz;

import java.util.EnumMap;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.wpilib.units.measure.Frequency;

public class SignalRatesTest {

  /** Stand-in groups. */
  public enum Groups implements SignalGroup {
    MOTION(100.0),
    THERMAL(4.0);

    private final double hertz;

    Groups(double hertz) {
      this.hertz = hertz;
    }

    @Override
    public Frequency defaultRate() {
      return Hertz.of(hertz);
    }
  }

  /** Stand-in signals. */
  public enum Signals implements Signal<Groups> {
    POSITION(Groups.MOTION),
    VELOCITY(Groups.MOTION),
    TEMPERATURE(Groups.THERMAL);

    private final Groups group;

    Signals(Groups group) {
      this.group = group;
    }

    @Override
    public Groups group() {
      return group;
    }
  }

  private static SignalRates<Signals, Groups> defaults() {
    return SignalRates.defaults(Signals.class, Groups.class);
  }

  @Test
  public void testDefaultsUseEachGroupsDefaultRate() {
    SignalRates<Signals, Groups> rates = defaults();

    assertEquals(Hertz.of(100.0), rates.rateOf(Groups.MOTION));
    assertEquals(Hertz.of(4.0), rates.rateOf(Groups.THERMAL));
    assertEquals(Hertz.of(100.0), rates.rateFor(Signals.POSITION));
    assertEquals(Hertz.of(4.0), rates.rateFor(Signals.TEMPERATURE));
    assertEquals(Map.of(), rates.signalOverrides());
  }

  @Test
  public void testWithGroupChangesEverySignalInTheGroup() {
    SignalRates<Signals, Groups> rates = defaults().withGroup(Groups.MOTION, Hertz.of(50.0));

    assertEquals(Hertz.of(50.0), rates.rateFor(Signals.POSITION));
    assertEquals(Hertz.of(50.0), rates.rateFor(Signals.VELOCITY));
    assertEquals(Hertz.of(4.0), rates.rateFor(Signals.TEMPERATURE));
    assertEquals(Hertz.of(100.0), defaults().rateFor(Signals.POSITION));
  }

  @Test
  public void testSignalOverridesBeatGroupRates() {
    SignalRates<Signals, Groups> rates =
        defaults()
            .withSignal(Signals.POSITION, Hertz.of(250.0))
            .withGroup(Groups.MOTION, Hertz.of(50.0));

    assertEquals(Hertz.of(250.0), rates.rateFor(Signals.POSITION));
    assertEquals(Hertz.of(50.0), rates.rateFor(Signals.VELOCITY));
  }

  @Test
  public void testOffTurnsAGroupOff() {
    SignalRates<Signals, Groups> rates = defaults().withGroup(Groups.THERMAL, SignalRates.OFF);

    assertEquals(0.0, rates.rateFor(Signals.TEMPERATURE).in(Hertz));
  }

  @Test
  public void testMapsAreCopied() {
    Map<Groups, Frequency> groupRates = new EnumMap<>(Groups.class);
    groupRates.put(Groups.MOTION, Hertz.of(100.0));
    groupRates.put(Groups.THERMAL, Hertz.of(4.0));
    SignalRates<Signals, Groups> rates =
        new SignalRates<>(Signals.class, Groups.class, groupRates, Map.of());
    groupRates.put(Groups.MOTION, Hertz.of(1.0));

    assertEquals(Hertz.of(100.0), rates.rateFor(Signals.POSITION));
    assertThrows(
        UnsupportedOperationException.class,
        () -> rates.groupRates().put(Groups.MOTION, Hertz.of(1.0)));
  }

  @Test
  public void shouldRejectMissingGroupRates() {
    assertThrows(
        IllegalArgumentException.class,
        () ->
            new SignalRates<>(
                Signals.class,
                Groups.class,
                Map.of(Groups.MOTION, Hertz.of(100.0)),
                Map.<Signals, Frequency>of()));
  }

  @Test
  public void shouldRejectNegativeRates() {
    assertThrows(
        IllegalArgumentException.class, () -> defaults().withGroup(Groups.MOTION, Hertz.of(-1.0)));
    assertThrows(
        IllegalArgumentException.class,
        () -> defaults().withSignal(Signals.POSITION, Hertz.of(-1.0)));
  }
}
