package edu.wpi.team190.gompeilib.sysid;

import java.util.Objects;
import java.util.function.Supplier;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;

/**
 * The travel a positional mechanism must stay inside while it is characterized. A leg stops once
 * the mechanism is within {@code margin} of the limit in its direction of travel; the other limit
 * is ignored, so a leg can start right next to it.
 *
 * <p>The margin is a fixed distance from the limit, read when the limit is checked. A dynamic leg
 * is still moving fast when it trips, so the margin has to cover the mechanism's stopping distance
 * from the step's top speed, not just its goal tolerance.
 *
 * @param position the mechanism's position now
 * @param lowerLimit the lowest position
 * @param upperLimit the highest position
 * @param margin how far short of a limit a leg stops
 * @param <P> the mechanism's position unit
 */
public record SysIdConstraint<P extends Unit>(
    Supplier<? extends Measure<P>> position,
    Measure<P> lowerLimit,
    Measure<P> upperLimit,
    Supplier<? extends Measure<P>> margin) {
  public SysIdConstraint {
    Objects.requireNonNull(position, "position");
    Objects.requireNonNull(lowerLimit, "lowerLimit");
    Objects.requireNonNull(upperLimit, "upperLimit");
    Objects.requireNonNull(margin, "margin");
  }

  /**
   * Returns whether the mechanism is within the margin of the limit in a direction.
   *
   * @param forward true to check the upper limit, false for the lower
   * @return true at or past that limit's margin
   */
  public boolean atLimit(boolean forward) {
    double position = this.position.get().baseUnitMagnitude();
    double margin = this.margin.get().baseUnitMagnitude();
    return forward
        ? position >= upperLimit.baseUnitMagnitude() - margin
        : position <= lowerLimit.baseUnitMagnitude() + margin;
  }
}
