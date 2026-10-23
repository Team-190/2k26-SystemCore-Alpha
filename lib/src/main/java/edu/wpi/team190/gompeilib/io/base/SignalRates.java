package edu.wpi.team190.gompeilib.io.base;

import static org.wpilib.units.Units.Hertz;

import edu.wpi.team190.gompeilib.GompeiLib;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Map;
import java.util.Set;
import org.wpilib.units.measure.Frequency;

/**
 * How often each field of a device's telemetry is published. Control-relevant fields run at {@code
 * control}; diagnostics run at {@code diagnostic}; {@code overrides} sets individual fields by
 * name. Each device's config says which of its fields are control-relevant.
 *
 * @param control the rate for control-relevant fields
 * @param diagnostic the rate for diagnostic fields
 * @param overrides per-field rates, keyed by telemetry field name
 */
public record SignalRates(
    Frequency control, Frequency diagnostic, Map<String, Frequency> overrides) {

  /**
   * The slowest control rate allowed. A hardware follower copies its leader's output signals, which
   * must be published at least this fast.
   */
  public static final Frequency MIN_CONTROL_RATE = Hertz.of(20.0);

  /** Loop-rate control signals and slow diagnostics. */
  public static final SignalRates DEFAULT =
      new SignalRates(Hertz.of(1.0 / GompeiLib.getLoopPeriod()), Hertz.of(4.0), Map.of());

  public SignalRates {
    if (control.lt(MIN_CONTROL_RATE)) {
      throw new IllegalArgumentException(
          "control rate must be at least " + MIN_CONTROL_RATE + ", got " + control);
    }
    overrides = Map.copyOf(overrides);
  }

  /**
   * Returns the rate a telemetry field is published at.
   *
   * @param field a telemetry field name
   * @param controlFields the device's control-relevant fields
   * @return the field's rate
   */
  public Frequency rateFor(String field, Set<String> controlFields) {
    Frequency override = overrides.get(field);
    if (override != null) {
      return override;
    }
    return controlFields.contains(field) ? control : diagnostic;
  }

  /**
   * Checks that every override names a field of the given telemetry class.
   *
   * @param telemetry the device's telemetry class
   * @throws IllegalArgumentException if an override names no such field
   */
  public void requireFieldsOf(Class<?> telemetry) {
    for (String name : overrides.keySet()) {
      boolean found = false;
      for (Field field : telemetry.getFields()) {
        found |= field.getName().equals(name) && !Modifier.isStatic(field.getModifiers());
      }
      if (!found) {
        throw new IllegalArgumentException(
            name + " is not a " + telemetry.getSimpleName() + " field");
      }
    }
  }
}
