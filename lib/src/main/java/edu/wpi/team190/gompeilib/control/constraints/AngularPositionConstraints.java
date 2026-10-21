package edu.wpi.team190.gompeilib.control.constraints;

import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.team190.gompeilib.tuning.GompeiTunables;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import lombok.Builder;
import org.wpilib.tunable.Tunable;
import org.wpilib.tunable.TunableTable;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularAccelerationUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;

/**
 * Tunable angular position constraints, published under {@code prefix} (for example {@code
 * /Tunables/<prefix>/Goal Tolerance}). Constraints built without a prefix are not published. Each
 * value is published and edited in the unit it was declared in, with its unit next to it. Unset
 * values default to zero, published in rotations, except the characterization margin, which
 * defaults to the goal tolerance.
 */
public final class AngularPositionConstraints
    implements Constraints.PositionConstraints<AngularPositionConstraints> {
  private final Tunable<Angle> goalTolerance;
  private final Tunable<AngularVelocity> maxVelocity;
  private final Tunable<AngularAcceleration> maxAcceleration;
  private final Tunable<Angle> characterizationMargin;

  private final List<Consumer<AngularPositionConstraints>> listeners = new ArrayList<>();
  private List<Double> lastNotifiedValues;

  @Builder(setterPrefix = "with")
  public AngularPositionConstraints(
      String prefix,
      Angle goalTolerance,
      AngularVelocity maxVelocity,
      AngularAcceleration maxAcceleration,
      Angle characterizationMargin) {
    TunableTable table = GompeiTunables.table(prefix);
    Runnable onTune = this::notifyListeners;
    Angle tolerance = Objects.requireNonNullElse(goalTolerance, Rotations.of(0));
    this.goalTolerance = GompeiTunables.value(table, "Goal Tolerance", tolerance, onTune);
    // Only characterization reads the margin, so editing it doesn't notify the profile listeners.
    this.characterizationMargin =
        GompeiTunables.value(
            table,
            "Characterization Margin",
            Objects.requireNonNullElse(characterizationMargin, tolerance),
            null);
    this.maxVelocity =
        GompeiTunables.value(
            table,
            "Max Velocity",
            Objects.requireNonNullElse(maxVelocity, RotationsPerSecond.of(0)),
            onTune);
    this.maxAcceleration =
        GompeiTunables.value(
            table,
            "Max Acceleration",
            Objects.requireNonNullElse(maxAcceleration, RotationsPerSecondPerSecond.of(0)),
            onTune);
    lastNotifiedValues = values();
  }

  public Tunable<Angle> goalTolerance() {
    return goalTolerance;
  }

  public Tunable<AngularVelocity> maxVelocity() {
    return maxVelocity;
  }

  public Tunable<AngularAcceleration> maxAcceleration() {
    return maxAcceleration;
  }

  /**
   * Returns how far short of a travel limit a characterization leg stops.
   *
   * @return the margin
   */
  public Tunable<Angle> characterizationMargin() {
    return characterizationMargin;
  }

  public double getGoalTolerance(AngleUnit unit) {
    return goalTolerance.get().in(unit);
  }

  public double getMaxVelocity(AngularVelocityUnit unit) {
    return maxVelocity.get().in(unit);
  }

  public double getMaxAcceleration(AngularAccelerationUnit unit) {
    return maxAcceleration.get().in(unit);
  }

  @Override
  public void onChange(Consumer<AngularPositionConstraints> listener) {
    listeners.add(listener);
  }

  private List<Double> values() {
    return List.of(
        goalTolerance.get().baseUnitMagnitude(),
        maxVelocity.get().baseUnitMagnitude(),
        maxAcceleration.get().baseUnitMagnitude());
  }

  private void notifyListeners() {
    // Every edited constraint triggers this; only notify once per distinct set of values.
    List<Double> values = values();
    if (values.equals(lastNotifiedValues)) {
      return;
    }
    lastNotifiedValues = values;
    listeners.forEach(listener -> listener.accept(this));
  }
}
