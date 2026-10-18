package edu.wpi.team190.gompeilib.math;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.Units;

public class MeasureUtilTest {

  @Test
  public void testAbs() {
    Measure<DistanceUnit> positive = Units.Meters.of(5.0);
    Measure<DistanceUnit> negative = Units.Meters.of(-5.0);
    Measure<DistanceUnit> zero = Units.Meters.of(0.0);

    assertEquals(5.0, MeasureUtil.abs(positive).in(Units.Meters), 1e-6);
    assertEquals(5.0, MeasureUtil.abs(negative).in(Units.Meters), 1e-6);
    assertEquals(0.0, MeasureUtil.abs(zero).in(Units.Meters), 1e-6);
  }

  @Test
  public void testConstructor() {
    assertNotNull(new MeasureUtil());
  }
}
