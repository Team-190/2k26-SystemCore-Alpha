package edu.wpi.team190.gompeilib.control;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.tuning.GompeiTunables;
import edu.wpi.team190.gompeilib.tuning.TunableMeasure;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Consumer;
import org.wpilib.tunable.TunableTable;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.LinearVelocityUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.MultUnit;
import org.wpilib.units.PerUnit;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.Unit;
import org.wpilib.units.measure.Per;

/**
 * A set of tunable feedback and feedforward gains, each a typed measure, published under {@code
 * prefix} (for example {@code /Tunables/<prefix>/kP}). Gains built without a prefix are not
 * published.
 *
 * <p>The gains are generic over three units:
 *
 * <ul>
 *   <li>{@code O}, the controller's output: volts, amps, or duty cycle for a motor slot, or a
 *       velocity for a chassis controller.
 *   <li>{@code Q}, the quantity the loop controls: an angle for an arm, a distance for an
 *       extension, an angular velocity for a flywheel.
 *   <li>{@code D}, the derivative unit {@code kV} and {@code kA} are per. It is set on its own
 *       rather than derived from {@code Q}: a flywheel uses the same unit for both, an arm doesn't.
 * </ul>
 *
 * <p>{@code kP} is output per {@code Q}, {@code kI} output per {@code Q}-second of integrated
 * error, {@code kD} output per rate of {@code Q}, {@code kS} and {@code kG} plain output, {@code
 * kV} output per {@code D}, and {@code kA} output per rate of {@code D}. A gain declared in the
 * wrong unit doesn't compile.
 *
 * <p>Each gain is published and edited in the unit it was declared in, with its unit next to it
 * (see {@link GompeiTunables}). A gain that isn't set is zero in the default unit the builder was
 * started with. Consumers read each gain in the unit they need: a simulation asks for gains per
 * radian, a Phoenix slot asks for them per rotation, and nobody writes a conversion factor.
 *
 * @param <O> the output unit
 * @param <Q> the controlled quantity's unit
 * @param <D> the derivative unit {@code kV} and {@code kA} are per
 */
public final class Gains<O extends Unit, Q extends Unit, D extends Unit> {
  /** One gain. */
  private enum Term {
    KP("kP", true),
    KI("kI", true),
    KD("kD", true),
    KS("kS", false),
    KV("kV", true),
    KA("kA", true),
    KG("kG", false);

    final String key;
    final boolean perControlled;

    Term(String key, boolean perControlled) {
      this.key = key;
      this.perControlled = perControlled;
    }
  }

  private final O outputUnit;
  private final Q controlledUnit;
  private final D derivativeUnit;

  // Exactly one of tunables (declared gains) and source (a view of other gains) is set.
  private final Map<Term, TunableMeasure<?>> tunables;
  private final Gains<O, ?, ?> source;
  private final double ratio;

  private final List<Consumer<Gains<O, Q, D>>> listeners = new ArrayList<>();
  private List<Double> lastNotifiedValues;

  private Gains(Builder<O, Q, D> builder) {
    outputUnit = builder.outputUnit;
    controlledUnit = builder.controlledUnit;
    derivativeUnit = builder.derivativeUnit;
    source = null;
    ratio = 1.0;

    TunableTable table = GompeiTunables.table(builder.prefix);
    Runnable onTune = this::notifyListeners;
    tunables = new EnumMap<>(Term.class);
    for (Term term : Term.values()) {
      Measure<?> initial =
          Objects.requireNonNullElseGet(builder.values.get(term), () -> defaultUnit(term).zero());
      tunables.put(term, GompeiTunables.measure(table, term.key, initial, onTune));
    }
    lastNotifiedValues = values();
  }

  private Gains(
      Gains<O, ?, ?> source,
      Q controlledUnit,
      D derivativeUnit,
      double sourceBaseUnitsPerBaseUnit) {
    this.outputUnit = source.outputUnit;
    this.controlledUnit = Objects.requireNonNull(controlledUnit, "controlledUnit");
    this.derivativeUnit = Objects.requireNonNull(derivativeUnit, "derivativeUnit");
    this.source = source;
    this.ratio = sourceBaseUnitsPerBaseUnit;
    this.tunables = null;
  }

  /**
   * Starts gains with the given default units. Gains left unset are zero in these units.
   *
   * @param outputUnit the output unit
   * @param controlledUnit the controlled quantity's unit
   * @param derivativeUnit the derivative unit {@code kV} and {@code kA} are per
   * @return the builder
   */
  public static <O extends Unit, Q extends Unit, D extends Unit> Builder<O, Q, D> builder(
      O outputUnit, Q controlledUnit, D derivativeUnit) {
    return new Builder<>(outputUnit, controlledUnit, derivativeUnit);
  }

  /**
   * Starts gains for a loop on angular position, defaulting to rotations like a Phoenix slot.
   *
   * @param outputUnit the output unit
   * @return the builder
   */
  public static <O extends Unit> Builder<O, AngleUnit, AngularVelocityUnit> angularPosition(
      O outputUnit) {
    return builder(outputUnit, Rotations, RotationsPerSecond);
  }

  /**
   * Starts gains for a loop on angular velocity, defaulting to rotations like a Phoenix slot.
   *
   * @param outputUnit the output unit
   * @return the builder
   */
  public static <O extends Unit>
      Builder<O, AngularVelocityUnit, AngularVelocityUnit> angularVelocity(O outputUnit) {
    return builder(outputUnit, RotationsPerSecond, RotationsPerSecond);
  }

  /**
   * Starts gains for a loop on linear position, defaulting to meters.
   *
   * @param outputUnit the output unit
   * @return the builder
   */
  public static <O extends Unit> Builder<O, DistanceUnit, LinearVelocityUnit> linearPosition(
      O outputUnit) {
    return builder(outputUnit, Meters, MetersPerSecond);
  }

  /**
   * Starts gains for a loop on linear velocity, defaulting to meters.
   *
   * @param outputUnit the output unit
   * @return the builder
   */
  public static <O extends Unit> Builder<O, LinearVelocityUnit, LinearVelocityUnit> linearVelocity(
      O outputUnit) {
    return builder(outputUnit, MetersPerSecond, MetersPerSecond);
  }

  /**
   * Returns the output unit the gains default to.
   *
   * @return the output unit
   */
  public O outputUnit() {
    return outputUnit;
  }

  /**
   * Returns the controlled quantity's unit the gains default to.
   *
   * @return the controlled unit
   */
  public Q controlledUnit() {
    return controlledUnit;
  }

  /**
   * Returns the derivative unit the gains default to.
   *
   * @return the derivative unit
   */
  public D derivativeUnit() {
    return derivativeUnit;
  }

  /**
   * Returns the base unit of the output, such as volts for millivolts.
   *
   * @return the output's base unit
   */
  // Every unit's base unit is an instance of its own class.
  @SuppressWarnings("unchecked")
  public O baseOutputUnit() {
    return (O) outputUnit.getBaseUnit();
  }

  public double getKP(PerUnit<O, Q> unit) {
    return in(Term.KP, unit);
  }

  public double getKI(PerUnit<O, ? extends MultUnit<Q, TimeUnit>> unit) {
    return in(Term.KI, unit);
  }

  public double getKD(PerUnit<O, ? extends PerUnit<Q, TimeUnit>> unit) {
    return in(Term.KD, unit);
  }

  public double getKS(O unit) {
    return in(Term.KS, unit);
  }

  public double getKV(PerUnit<O, D> unit) {
    return in(Term.KV, unit);
  }

  public double getKA(PerUnit<O, ? extends PerUnit<D, TimeUnit>> unit) {
    return in(Term.KA, unit);
  }

  public double getKG(O unit) {
    return in(Term.KG, unit);
  }

  /**
   * Returns a live, unpublished view of these gains for the same mechanism measured in other units,
   * where one base unit of the new quantity is {@code sourceBaseUnitsPerBaseUnit} base units of
   * this one. A drum of radius r turns one radian into r meters, so meters-based gains viewed per
   * radian use r. The view reads these gains on every call and notifies its listeners when they
   * change.
   *
   * @param controlledUnit the new controlled quantity's unit
   * @param derivativeUnit the new derivative unit
   * @param sourceBaseUnitsPerBaseUnit base units of this quantity per base unit of the new one
   * @return the view
   * @throws IllegalArgumentException if the ratio isn't positive and finite
   */
  public <Q2 extends Unit, D2 extends Unit> Gains<O, Q2, D2> view(
      Q2 controlledUnit, D2 derivativeUnit, double sourceBaseUnitsPerBaseUnit) {
    if (!(sourceBaseUnitsPerBaseUnit > 0.0) || Double.isInfinite(sourceBaseUnitsPerBaseUnit)) {
      throw new IllegalArgumentException(
          "Ratio must be positive and finite, was " + sourceBaseUnitsPerBaseUnit);
    }
    return new Gains<>(this, controlledUnit, derivativeUnit, sourceBaseUnitsPerBaseUnit);
  }

  /**
   * Registers a callback to run when any gain is edited from a dashboard. The callback runs once
   * per batch of edits, after all edits in that batch have been applied.
   *
   * @param listener callback receiving these gains
   */
  public void onChange(Consumer<Gains<O, Q, D>> listener) {
    if (source != null) {
      source.onChange(changed -> listener.accept(this));
    } else {
      listeners.add(listener);
    }
  }

  private double in(Term term, Unit unit) {
    return unit.fromBaseUnits(baseMagnitude(term));
  }

  private double baseMagnitude(Term term) {
    if (source != null) {
      double value = source.baseMagnitude(term);
      return term.perControlled ? value * ratio : value;
    }
    return tunables.get(term).get().baseUnitMagnitude();
  }

  private Unit defaultUnit(Term term) {
    return switch (term) {
      case KP -> PerUnit.combine(outputUnit, controlledUnit);
      case KI -> PerUnit.combine(outputUnit, MultUnit.combine(controlledUnit, Seconds));
      case KD -> PerUnit.combine(outputUnit, controlledUnit.per(Seconds));
      case KS, KG -> outputUnit;
      case KV -> PerUnit.combine(outputUnit, derivativeUnit);
      case KA -> PerUnit.combine(outputUnit, derivativeUnit.per(Seconds));
    };
  }

  private List<Double> values() {
    List<Double> values = new ArrayList<>();
    for (Term term : Term.values()) {
      values.add(baseMagnitude(term));
    }
    return values;
  }

  private void notifyListeners() {
    // Every edited gain triggers this; only notify once per distinct set of values.
    List<Double> values = values();
    if (values.equals(lastNotifiedValues)) {
      return;
    }
    lastNotifiedValues = values;
    listeners.forEach(listener -> listener.accept(this));
  }

  /**
   * Builds {@link Gains}. Each setter takes the gain as a measure in the unit it was tuned in.
   *
   * @param <O> the output unit
   * @param <Q> the controlled quantity's unit
   * @param <D> the derivative unit {@code kV} and {@code kA} are per
   */
  public static final class Builder<O extends Unit, Q extends Unit, D extends Unit> {
    private final O outputUnit;
    private final Q controlledUnit;
    private final D derivativeUnit;
    private final Map<Term, Measure<?>> values = new EnumMap<>(Term.class);
    private String prefix;

    private Builder(O outputUnit, Q controlledUnit, D derivativeUnit) {
      this.outputUnit = Objects.requireNonNull(outputUnit, "outputUnit");
      this.controlledUnit = Objects.requireNonNull(controlledUnit, "controlledUnit");
      this.derivativeUnit = Objects.requireNonNull(derivativeUnit, "derivativeUnit");
    }

    /**
     * Sets where the gains are published, or null to leave them unpublished.
     *
     * @param prefix the table path
     * @return this builder
     */
    public Builder<O, Q, D> withPrefix(String prefix) {
      this.prefix = prefix;
      return this;
    }

    public Builder<O, Q, D> withKP(Per<O, Q> kP) {
      return set(Term.KP, kP);
    }

    public Builder<O, Q, D> withKI(Per<O, ? extends MultUnit<Q, TimeUnit>> kI) {
      return set(Term.KI, kI);
    }

    public Builder<O, Q, D> withKD(Per<O, ? extends PerUnit<Q, TimeUnit>> kD) {
      return set(Term.KD, kD);
    }

    public Builder<O, Q, D> withKS(Measure<O> kS) {
      return set(Term.KS, kS);
    }

    public Builder<O, Q, D> withKV(Per<O, D> kV) {
      return set(Term.KV, kV);
    }

    public Builder<O, Q, D> withKA(Per<O, ? extends PerUnit<D, TimeUnit>> kA) {
      return set(Term.KA, kA);
    }

    public Builder<O, Q, D> withKG(Measure<O> kG) {
      return set(Term.KG, kG);
    }

    public Gains<O, Q, D> build() {
      return new Gains<>(this);
    }

    private Builder<O, Q, D> set(Term term, Measure<?> value) {
      values.put(term, Objects.requireNonNull(value, term.key));
      return this;
    }
  }
}
