package edu.wpi.team190.gompeilib.core.utility;

import org.wpilib.units.Measure;
import org.wpilib.units.Unit;

public class ExtensionMethods {
  public static <U extends Unit> Measure<U> abs(Measure<U> measure) {
    // U measure_unit = measure.baseUnit();
    // double measure_value = measure.baseUnitMagnitude();
    // double absoluteDouble = Math.abs(measure_value);
    // Measure<U> absoluteMeasure = measure_unit.of(absoluteDouble);
    // return absoluteMeasure;
    return measure.times(Math.signum(measure.baseUnitMagnitude()));
  }
}
