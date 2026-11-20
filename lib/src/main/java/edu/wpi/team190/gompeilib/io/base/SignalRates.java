package edu.wpi.team190.gompeilib.io.base;

import static org.wpilib.units.Units.Hertz;

import java.util.Collections;
import java.util.EnumMap;
import java.util.Map;
import org.wpilib.units.measure.Frequency;

/**
 * How often each of a device's signals is published. Every signal belongs to one of the device's
 * {@link SignalGroup}s, and each group has its own rate; {@code signalOverrides} sets individual
 * signals regardless of their group. A rate of zero turns a signal off.
 *
 * @param signals the device's signal enum
 * @param groups the device's group enum
 * @param groupRates the rate of every group
 * @param signalOverrides per-signal rates
 * @param <S> the device's signal enum
 * @param <G> the device's group enum
 */
public record SignalRates<S extends Enum<S> & Signal<G>, G extends Enum<G> & SignalGroup>(
    Class<S> signals,
    Class<G> groups,
    Map<G, Frequency> groupRates,
    Map<S, Frequency> signalOverrides) {

  /** The rate that turns a signal off. */
  public static final Frequency OFF = Hertz.of(0.0);

  public SignalRates {
    for (G group : groups.getEnumConstants()) {
      Frequency rate = groupRates.get(group);
      if (rate == null) {
        throw new IllegalArgumentException("no rate for group " + group);
      }
      requireNonNegative(group, rate);
    }
    signalOverrides.forEach(SignalRates::requireNonNegative);
    groupRates = copy(groups, groupRates);
    signalOverrides = copy(signals, signalOverrides);
  }

  /**
   * Returns every group at its default rate, with no signal overrides.
   *
   * @param signals the device's signal enum
   * @param groups the device's group enum
   * @param <S> the device's signal enum
   * @param <G> the device's group enum
   * @return the default rates
   */
  public static <S extends Enum<S> & Signal<G>, G extends Enum<G> & SignalGroup>
      SignalRates<S, G> defaults(Class<S> signals, Class<G> groups) {
    Map<G, Frequency> rates = new EnumMap<>(groups);
    for (G group : groups.getEnumConstants()) {
      rates.put(group, group.defaultRate());
    }
    return new SignalRates<>(signals, groups, rates, Map.of());
  }

  /**
   * Returns a copy with one group's rate changed.
   *
   * @param group the group
   * @param rate its rate, or {@link #OFF}
   * @return the new rates
   */
  public SignalRates<S, G> withGroup(G group, Frequency rate) {
    Map<G, Frequency> rates = new EnumMap<>(groupRates);
    rates.put(group, rate);
    return new SignalRates<>(signals, groups, rates, signalOverrides);
  }

  /**
   * Returns a copy with one signal's rate overridden, regardless of its group's rate.
   *
   * @param signal the signal
   * @param rate its rate, or {@link #OFF}
   * @return the new rates
   */
  public SignalRates<S, G> withSignal(S signal, Frequency rate) {
    Map<S, Frequency> overrides = new EnumMap<>(signals);
    overrides.putAll(signalOverrides);
    overrides.put(signal, rate);
    return new SignalRates<>(signals, groups, groupRates, overrides);
  }

  /**
   * Returns a group's rate.
   *
   * @param group the group
   * @return its rate
   */
  public Frequency rateOf(G group) {
    return groupRates.get(group);
  }

  /**
   * Returns the rate a signal is published at: its override, or else its group's rate.
   *
   * @param signal the signal
   * @return its rate
   */
  public Frequency rateFor(S signal) {
    Frequency override = signalOverrides.get(signal);
    return override != null ? override : groupRates.get(signal.group());
  }

  private static <K extends Enum<K>> Map<K, Frequency> copy(Class<K> type, Map<K, Frequency> map) {
    Map<K, Frequency> copy = new EnumMap<>(type);
    copy.putAll(map);
    return Collections.unmodifiableMap(copy);
  }

  private static void requireNonNegative(Object name, Frequency rate) {
    if (rate.in(Hertz) < 0.0) {
      throw new IllegalArgumentException(name + " rate must not be negative, got " + rate);
    }
  }
}
