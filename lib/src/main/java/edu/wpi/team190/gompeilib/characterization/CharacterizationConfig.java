package edu.wpi.team190.gompeilib.characterization;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.tuning.GompeiTunables;
import java.util.EnumSet;
import java.util.Objects;
import java.util.Set;
import lombok.Builder;
import lombok.Getter;
import org.wpilib.tunable.Tunable;
import org.wpilib.tunable.TunableTable;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.Unit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Per;
import org.wpilib.units.measure.Time;

/**
 * How a mechanism is characterized for one open-loop output: the quasistatic ramp, the dynamic
 * step, each leg's timeout, the settle time between legs, and which legs run. The ramp, step,
 * timeouts and settle time are tunable, published under {@code prefix} (for example {@code
 * /Tunables/<prefix>/Step Output}); a config built without a prefix is not published. The routine
 * reads each value when a leg (or settle) starts, so an edit takes effect on the next one.
 *
 * <p>Start from {@link #voltage()} or {@link #current()}, which set the output unit.
 *
 * @param <U> the output unit
 */
public final class CharacterizationConfig<U extends Unit> {
  /** The settle time used when none is set. */
  public static final Time DEFAULT_SETTLE_TIME = Seconds.of(1);

  /** One leg of a characterization routine, in the order the routine runs them. */
  public enum Leg {
    QUASISTATIC_FORWARD(true, true),
    QUASISTATIC_REVERSE(true, false),
    DYNAMIC_FORWARD(false, true),
    DYNAMIC_REVERSE(false, false);

    private final boolean quasistatic;
    private final boolean forward;

    Leg(boolean quasistatic, boolean forward) {
      this.quasistatic = quasistatic;
      this.forward = forward;
    }

    /**
     * Returns whether this leg ramps the output rather than stepping it.
     *
     * @return true for a quasistatic leg
     */
    public boolean isQuasistatic() {
      return quasistatic;
    }

    /**
     * Returns whether this leg drives in the positive direction.
     *
     * @return true for a forward leg
     */
    public boolean isForward() {
      return forward;
    }
  }

  @Getter private final U outputUnit;
  private final Tunable<Per<U, TimeUnit>> rampRate;
  private final Tunable<Measure<U>> stepOutput;
  private final Tunable<Time> quasistaticTimeout;
  private final Tunable<Time> dynamicTimeout;
  private final Tunable<Time> settleTime;
  @Getter private final Set<Leg> legs;

  /**
   * Creates a characterization config.
   *
   * @param prefix where to publish the tunables, or null to leave them unpublished
   * @param outputUnit the output unit
   * @param rampRate how fast the quasistatic legs ramp the output
   * @param stepOutput the output the dynamic legs step to
   * @param quasistaticTimeout the longest a quasistatic leg runs
   * @param dynamicTimeout the longest a dynamic leg runs
   * @param settleTime how long to wait at zero output between legs, or null for {@link
   *     #DEFAULT_SETTLE_TIME}
   * @param legs which legs run, or null for all four
   */
  @Builder(setterPrefix = "with")
  public CharacterizationConfig(
      String prefix,
      U outputUnit,
      Per<U, TimeUnit> rampRate,
      Measure<U> stepOutput,
      Time quasistaticTimeout,
      Time dynamicTimeout,
      Time settleTime,
      Set<Leg> legs) {
    this.outputUnit = Objects.requireNonNull(outputUnit, "outputUnit");
    TunableTable table = GompeiTunables.table(prefix);
    this.rampRate =
        GompeiTunables.value(
            table, "Ramp Rate", Objects.requireNonNull(rampRate, "rampRate"), null);
    this.stepOutput =
        GompeiTunables.value(
            table, "Step Output", Objects.requireNonNull(stepOutput, "stepOutput"), null);
    this.quasistaticTimeout =
        GompeiTunables.value(
            table,
            "Quasistatic Timeout",
            Objects.requireNonNull(quasistaticTimeout, "quasistaticTimeout"),
            null);
    this.dynamicTimeout =
        GompeiTunables.value(
            table,
            "Dynamic Timeout",
            Objects.requireNonNull(dynamicTimeout, "dynamicTimeout"),
            null);
    this.settleTime =
        GompeiTunables.value(
            table,
            "Settle Time",
            Objects.requireNonNullElse(settleTime, DEFAULT_SETTLE_TIME),
            null);
    this.legs =
        legs == null || legs.isEmpty()
            ? Set.copyOf(EnumSet.allOf(Leg.class))
            : Set.copyOf(EnumSet.copyOf(legs));
  }

  /**
   * Starts a config that characterizes in voltage.
   *
   * @return the builder
   */
  public static CharacterizationConfigBuilder<VoltageUnit> voltage() {
    return CharacterizationConfig.<VoltageUnit>builder().withOutputUnit(Volts);
  }

  /**
   * Starts a config that characterizes in torque current.
   *
   * @return the builder
   */
  public static CharacterizationConfigBuilder<CurrentUnit> current() {
    return CharacterizationConfig.<CurrentUnit>builder().withOutputUnit(Amps);
  }

  public Tunable<Per<U, TimeUnit>> rampRate() {
    return rampRate;
  }

  public Tunable<Measure<U>> stepOutput() {
    return stepOutput;
  }

  public Tunable<Time> quasistaticTimeout() {
    return quasistaticTimeout;
  }

  public Tunable<Time> dynamicTimeout() {
    return dynamicTimeout;
  }

  public Tunable<Time> settleTime() {
    return settleTime;
  }

  /**
   * Returns the timeout for a leg.
   *
   * @param leg the leg
   * @return its timeout now
   */
  public Time timeout(Leg leg) {
    return (leg.isQuasistatic() ? quasistaticTimeout : dynamicTimeout).get();
  }
}
