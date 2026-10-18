package edu.wpi.team190.gompeilib.math;

import org.wpilib.units.Measure;

public class MeasureUtil {
  @SuppressWarnings("unchecked")
  public static <M extends Measure<?>> M abs(M measure) {
    return measure.baseUnitMagnitude() < 0 ? (M) measure.unaryMinus() : measure;
  }
}
