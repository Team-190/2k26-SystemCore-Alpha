package edu.wpi.team190.gompeilib.io.base;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Hertz;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.wpilib.units.measure.Frequency;

public class SignalRatesTest {
  private static final Set<String> CONTROL = Set.of("position");

  /** A stand-in telemetry class. */
  public static class Telemetry {
    public static int notAField = 0;
    public double position;
    public double temperature;
  }

  @Test
  public void testDefaultRates() {
    assertEquals(Hertz.of(50.0), SignalRates.DEFAULT.control());
    assertEquals(Hertz.of(4.0), SignalRates.DEFAULT.diagnostic());
    assertEquals(Map.of(), SignalRates.DEFAULT.overrides());
  }

  @Test
  public void testRateForControlAndDiagnosticFields() {
    assertEquals(Hertz.of(50.0), SignalRates.DEFAULT.rateFor("position", CONTROL));
    assertEquals(Hertz.of(4.0), SignalRates.DEFAULT.rateFor("temperature", CONTROL));
  }

  @Test
  public void testOverridesWin() {
    SignalRates rates =
        new SignalRates(
            Hertz.of(100.0),
            Hertz.of(10.0),
            Map.of("temperature", Hertz.of(1.0), "position", Hertz.of(250.0)));

    assertEquals(Hertz.of(1.0), rates.rateFor("temperature", CONTROL));
    assertEquals(Hertz.of(250.0), rates.rateFor("position", CONTROL));
    assertEquals(Hertz.of(100.0), rates.rateFor("velocity", Set.of("velocity")));
    assertEquals(Hertz.of(10.0), rates.rateFor("voltage", CONTROL));
  }

  @Test
  public void testOverridesAreCopied() {
    Map<String, Frequency> overrides = new HashMap<>();
    overrides.put("position", Hertz.of(200.0));
    SignalRates rates = new SignalRates(Hertz.of(50.0), Hertz.of(4.0), overrides);
    overrides.clear();

    assertEquals(Hertz.of(200.0), rates.rateFor("position", CONTROL));
  }

  @Test
  public void testMinimumControlRateIsAccepted() {
    assertDoesNotThrow(
        () -> new SignalRates(SignalRates.MIN_CONTROL_RATE, Hertz.of(4.0), Map.of()));
  }

  @Test
  public void shouldRejectSlowControlRate() {
    assertThrows(
        IllegalArgumentException.class,
        () -> new SignalRates(Hertz.of(19.0), Hertz.of(4.0), Map.of()));
  }

  @Test
  public void testRequireFieldsOf() {
    SignalRates known =
        new SignalRates(Hertz.of(50.0), Hertz.of(4.0), Map.of("temperature", Hertz.of(1.0)));
    SignalRates unknown =
        new SignalRates(Hertz.of(50.0), Hertz.of(4.0), Map.of("speed", Hertz.of(1.0)));
    SignalRates staticField =
        new SignalRates(Hertz.of(50.0), Hertz.of(4.0), Map.of("notAField", Hertz.of(1.0)));

    assertDoesNotThrow(() -> known.requireFieldsOf(Telemetry.class));
    assertThrows(IllegalArgumentException.class, () -> unknown.requireFieldsOf(Telemetry.class));
    assertThrows(
        IllegalArgumentException.class, () -> staticField.requireFieldsOf(Telemetry.class));
  }
}
