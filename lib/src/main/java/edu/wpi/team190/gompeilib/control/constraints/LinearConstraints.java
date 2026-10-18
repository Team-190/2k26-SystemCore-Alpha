package edu.wpi.team190.gompeilib.control.constraints;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;

import edu.wpi.team190.gompeilib.tuning.GompeiTunables;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import lombok.Builder;
import org.wpilib.tunable.Tunable;
import org.wpilib.tunable.TunableTable;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.LinearAccelerationUnit;
import org.wpilib.units.LinearVelocityUnit;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.LinearVelocity;

/**
 * Tunable linear motion constraints, published under {@code prefix} (for example {@code
 * /Tunables/<prefix>/Goal Tolerance}). Constraints built without a prefix are not published. Unset
 * values default to zero, except the SysId margin, which defaults to the goal tolerance.
 */
public final class LinearConstraints implements Constraints.PositionConstraints<LinearConstraints> {
  private final Tunable<Distance> goalTolerance;
  private final Tunable<LinearVelocity> maxVelocity;
  private final Tunable<LinearAcceleration> maxAcceleration;
  private final Tunable<Distance> sysIdMargin;

  private final List<Consumer<LinearConstraints>> listeners = new ArrayList<>();
  private List<Double> lastNotifiedValues;

  @Builder(setterPrefix = "with")
  public LinearConstraints(
      String prefix,
      Distance goalTolerance,
      LinearVelocity maxVelocity,
      LinearAcceleration maxAcceleration,
      Distance sysIdMargin) {
    TunableTable table = GompeiTunables.table(prefix);
    Runnable onTune = this::notifyListeners;
    Distance tolerance = Objects.requireNonNullElse(goalTolerance, Meters.of(0));
    this.goalTolerance = GompeiTunables.value(table, "Goal Tolerance", tolerance, onTune);
    // Only SysId reads the margin, so editing it doesn't notify the profile listeners.
    this.sysIdMargin =
        GompeiTunables.value(
            table, "SysId Margin", Objects.requireNonNullElse(sysIdMargin, tolerance), null);
    this.maxVelocity =
        GompeiTunables.value(
            table,
            "Max Velocity",
            Objects.requireNonNullElse(maxVelocity, MetersPerSecond.of(0)),
            onTune);
    this.maxAcceleration =
        GompeiTunables.value(
            table,
            "Max Acceleration",
            Objects.requireNonNullElse(maxAcceleration, MetersPerSecondPerSecond.of(0)),
            onTune);
    lastNotifiedValues = values();
  }

  public Tunable<Distance> goalTolerance() {
    return goalTolerance;
  }

  public Tunable<LinearVelocity> maxVelocity() {
    return maxVelocity;
  }

  public Tunable<LinearAcceleration> maxAcceleration() {
    return maxAcceleration;
  }

  /**
   * Returns how far short of a travel limit a SysId leg stops.
   *
   * @return the margin
   */
  public Tunable<Distance> sysIdMargin() {
    return sysIdMargin;
  }

  public double getGoalToleranceMeters(DistanceUnit unit) {
    return goalTolerance.get().in(unit);
  }

  public double getMaxVelocityMetersPerSecond(LinearVelocityUnit unit) {
    return maxVelocity.get().in(unit);
  }

  public double getMaxAccelerationMetersPerSecondSquared(LinearAccelerationUnit unit) {
    return maxAcceleration.get().in(unit);
  }

  @Override
  public void onChange(Consumer<LinearConstraints> listener) {
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
