package edu.wpi.team190.gompeilib.core.utility;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.Units;

public class ExtensionMethodsTest {

  @Test
  public void testAbs() {
    Measure<DistanceUnit> positive = Units.Meters.of(5.0);
    Measure<DistanceUnit> negative = Units.Meters.of(-5.0);
    Measure<DistanceUnit> zero = Units.Meters.of(0.0);

    assertEquals(5.0, ExtensionMethods.abs(positive).in(Units.Meters), 1e-6);
    assertEquals(5.0, ExtensionMethods.abs(negative).in(Units.Meters), 1e-6);
    assertEquals(0.0, ExtensionMethods.abs(zero).in(Units.Meters), 1e-6);
  }

  @Test
  public void testMatchesSetpoint() {
    Setpoint<DistanceUnit> setpoint = new Setpoint<>(Units.Meters.of(10.0), Units.Meters.of(1.0));
    Measure<DistanceUnit> matching = Units.Meters.of(10.0);
    Measure<DistanceUnit> matchingEquivalent = Units.Millimeters.of(10000.0);
    Measure<DistanceUnit> different = Units.Meters.of(5.0);

    assertTrue(ExtensionMethods.matchesSetpoint(matching, setpoint));
    assertTrue(ExtensionMethods.matchesSetpoint(matchingEquivalent, setpoint));
    assertFalse(ExtensionMethods.matchesSetpoint(different, setpoint));
  }
}
