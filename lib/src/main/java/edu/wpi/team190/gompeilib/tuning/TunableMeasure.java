package edu.wpi.team190.gompeilib.tuning;

import java.util.Objects;
import org.wpilib.tunable.Tunable;
import org.wpilib.tunable.TunableConfig;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.measure.Per;

/**
 * A tunable measure published as a plain number in the unit it was declared in, rather than in its
 * base unit. A hood velocity declared as {@code RotationsPerSecond.of(2)} shows {@code 2.0}, and a
 * dashboard edit is read in rotations per second. Use {@link GompeiTunables#measure} to create one;
 * it also publishes the unit next to the value.
 *
 * @param <M> the measure type
 */
public final class TunableMeasure<M extends Measure<?>> extends Tunable<M>
    implements Tunable.CustomTunable {
  private final Unit unit;
  private final Class<M> type;
  private final TunableDouble magnitude;
  private M value;

  /**
   * Creates a tunable measure in the initial value's unit.
   *
   * @param initial the initial value, whose unit is the declared unit
   * @param config the tunable config
   */
  @SuppressWarnings("unchecked")
  TunableMeasure(M initial, TunableConfig config) {
    super(config);
    this.value = Objects.requireNonNull(initial, "initial");
    this.unit = initial.unit();
    this.type = (Class<M>) initial.getClass();
    of(initial.magnitude());
    this.magnitude =
        new TunableDouble(config, true) {
          @Override
          public void set(double magnitude) {
            value = of(magnitude);
            markChanged();
          }

          @Override
          public double get() {
            return value.magnitude();
          }
        };
  }

  /**
   * Returns the unit the value is published and edited in.
   *
   * @return the declared unit
   */
  public Unit unit() {
    return unit;
  }

  @Override
  public void set(M value) {
    if (!unit.getBaseUnit().equivalent(value.unit().getBaseUnit())) {
      throw new IllegalArgumentException(
          "Measure unit " + value.unit() + " is not compatible with " + unit);
    }
    magnitude.set(unit.fromBaseUnits(value.baseUnitMagnitude()));
  }

  @Override
  public M get() {
    return value;
  }

  @Override
  public TunableDouble getInnerTunable() {
    return magnitude;
  }

  @Override
  public boolean hasChanged() {
    return magnitude.hasChanged();
  }

  @Override
  public boolean supportsChangeNotification() {
    return magnitude.supportsChangeNotification();
  }

  @Override
  public void resetChanged() {
    magnitude.resetChanged();
  }

  @Override
  public Class<M> getTypeClass() {
    return type;
  }

  // A Per built with ofNative stays a Per; any other measure is its unit's own measure type.
  @SuppressWarnings("unchecked")
  private M of(double magnitude) {
    Measure<?> measure =
        value instanceof Per<?, ?> per ? per.unit().ofNative(magnitude) : unit.of(magnitude);
    if (!type.isInstance(measure)) {
      throw new IllegalArgumentException(
          "Can't rebuild a " + type.getSimpleName() + " from its unit " + unit);
    }
    return (M) measure;
  }
}
