package edu.wpi.team190.gompeilib.io.motor.outputs;

import java.util.Optional;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.DimensionlessUnit;
import org.wpilib.units.Unit;
import org.wpilib.units.VoltageUnit;

/** The quantity a motor request drives the output with. */
public enum OutputType {
  DUTY_CYCLE,
  VOLTAGE,
  CURRENT;

  /**
   * Returns the output type a unit measures: duty cycle for a dimensionless unit, voltage for a
   * voltage unit, and current for a current unit.
   *
   * @param unit the unit
   * @return the output type, or empty if a motor can't output that unit
   */
  public static Optional<OutputType> of(Unit unit) {
    return switch (unit) {
      case DimensionlessUnit _ -> Optional.of(DUTY_CYCLE);
      case VoltageUnit _ -> Optional.of(VOLTAGE);
      case CurrentUnit _ -> Optional.of(CURRENT);
      default -> Optional.empty();
    };
  }
}
