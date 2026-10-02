package edu.wpi.team190.gompeilib.core.utility;

import org.wpilib.units.Measure;
import org.wpilib.units.Unit;

public class ExtensionMethods {
  public static <U extends Unit> Measure<U> abs(Measure<U> measure) {
    return measure.times(Math.signum(measure.baseUnitMagnitude()));
  }
}
