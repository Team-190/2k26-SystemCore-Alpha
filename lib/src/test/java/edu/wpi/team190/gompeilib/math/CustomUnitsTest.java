package edu.wpi.team190.gompeilib.math;

import static edu.wpi.team190.gompeilib.math.CustomUnits.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import org.junit.jupiter.api.Test;

public class CustomUnitsTest {
  @Test
  public void testCustomUnits() throws Exception {
    var constructor = CustomUnits.class.getDeclaredConstructor();
    constructor.setAccessible(true);
    constructor.newInstance();

    // Ramp rates
    assertNotNull(AmpsPerSecond);
    assertSame(AmpsPerSecond, ampsPerSecond);
    assertNotNull(VoltsPerSecond);
    assertSame(VoltsPerSecond, voltsPerSecond);
    assertNotNull(PercentPerSecond);
    assertSame(PercentPerSecond, percentPerSecond);

    // Jerk
    assertNotNull(RotationsPerSecondCubed);
    assertSame(RotationsPerSecondCubed, rotationsPerSecondCubed);
    assertNotNull(RadiansPerSecondCubed);
    assertSame(RadiansPerSecondCubed, radiansPerSecondCubed);
    assertNotNull(DegreesPerSecondCubed);
    assertSame(DegreesPerSecondCubed, degreesPerSecondCubed);
    assertNotNull(MetersPerSecondCubed);
    assertSame(MetersPerSecondCubed, metersPerSecondCubed);

    // Angular (Duty Cycle / Percent)
    assertNotNull(PercentPerRadian);
    assertSame(PercentPerRadian, percentPerRadian);
    assertNotNull(PercentPerRotation);
    assertSame(PercentPerRotation, percentPerRotation);
    assertNotNull(PercentPerDegree);
    assertSame(PercentPerDegree, percentPerDegree);

    // Angular (Voltage)
    assertNotNull(VoltsPerRadian);
    assertSame(VoltsPerRadian, voltsPerRadian);
    assertNotNull(VoltsPerRotation);
    assertSame(VoltsPerRotation, voltsPerRotation);
    assertNotNull(VoltsPerDegree);
    assertSame(VoltsPerDegree, voltsPerDegree);
    assertNotNull(VoltsPerRadianPerSecond);
    assertSame(VoltsPerRadianPerSecond, voltsPerRadianPerSecond);
    assertNotNull(VoltsPerRotationPerSecond);
    assertSame(VoltsPerRotationPerSecond, voltsPerRotationPerSecond);
    assertNotNull(VoltsPerDegreePerSecond);
    assertSame(VoltsPerDegreePerSecond, voltsPerDegreePerSecond);
    assertNotNull(VoltsPerRadianPerSecondSquared);
    assertSame(VoltsPerRadianPerSecondSquared, voltsPerRadianPerSecondSquared);
    assertNotNull(VoltsPerRotationPerSecondSquared);
    assertSame(VoltsPerRotationPerSecondSquared, voltsPerRotationPerSecondSquared);
    assertNotNull(VoltsPerDegreePerSecondSquared);
    assertSame(VoltsPerDegreePerSecondSquared, voltsPerDegreePerSecondSquared);
    assertNotNull(VoltsPerRadianSecond);
    assertSame(VoltsPerRadianSecond, voltsPerRadianSecond);
    assertNotNull(VoltsPerRotationSecond);
    assertSame(VoltsPerRotationSecond, voltsPerRotationSecond);

    // Angular (Current)
    assertNotNull(AmpsPerRadian);
    assertSame(AmpsPerRadian, ampsPerRadian);
    assertNotNull(AmpsPerRotation);
    assertSame(AmpsPerRotation, ampsPerRotation);
    assertNotNull(AmpsPerDegree);
    assertSame(AmpsPerDegree, ampsPerDegree);
    assertNotNull(AmpsPerRadianPerSecond);
    assertSame(AmpsPerRadianPerSecond, ampsPerRadianPerSecond);
    assertNotNull(AmpsPerRotationPerSecond);
    assertSame(AmpsPerRotationPerSecond, ampsPerRotationPerSecond);
    assertNotNull(AmpsPerDegreePerSecond);
    assertSame(AmpsPerDegreePerSecond, ampsPerDegreePerSecond);
    assertNotNull(AmpsPerRadianPerSecondSquared);
    assertSame(AmpsPerRadianPerSecondSquared, ampsPerRadianPerSecondSquared);
    assertNotNull(AmpsPerRotationPerSecondSquared);
    assertSame(AmpsPerRotationPerSecondSquared, ampsPerRotationPerSecondSquared);
    assertNotNull(AmpsPerDegreePerSecondSquared);
    assertSame(AmpsPerDegreePerSecondSquared, ampsPerDegreePerSecondSquared);
    assertNotNull(AmpsPerRadianSecond);
    assertSame(AmpsPerRadianSecond, ampsPerRadianSecond);
    assertNotNull(AmpsPerRotationSecond);
    assertSame(AmpsPerRotationSecond, ampsPerRotationSecond);

    // Linear (Voltage)
    assertNotNull(VoltsPerMeter);
    assertSame(VoltsPerMeter, voltsPerMeter);
    assertNotNull(VoltsPerInch);
    assertSame(VoltsPerInch, voltsPerInch);
    assertNotNull(VoltsPerMeterPerSecond);
    assertSame(VoltsPerMeterPerSecond, voltsPerMeterPerSecond);
    assertNotNull(VoltsPerInchPerSecond);
    assertSame(VoltsPerInchPerSecond, voltsPerInchPerSecond);
    assertNotNull(VoltsPerMeterPerSecondSquared);
    assertSame(VoltsPerMeterPerSecondSquared, voltsPerMeterPerSecondSquared);
    assertNotNull(VoltsPerMeterSecond);
    assertSame(VoltsPerMeterSecond, voltsPerMeterSecond);

    // Linear (Current)
    assertNotNull(AmpsPerMeter);
    assertSame(AmpsPerMeter, ampsPerMeter);
    assertNotNull(AmpsPerInch);
    assertSame(AmpsPerInch, ampsPerInch);
    assertNotNull(AmpsPerMeterPerSecond);
    assertSame(AmpsPerMeterPerSecond, ampsPerMeterPerSecond);
    assertNotNull(AmpsPerInchPerSecond);
    assertSame(AmpsPerInchPerSecond, ampsPerInchPerSecond);
    assertNotNull(AmpsPerMeterPerSecondSquared);
    assertSame(AmpsPerMeterPerSecondSquared, ampsPerMeterPerSecondSquared);
    assertNotNull(AmpsPerMeterSecond);
    assertSame(AmpsPerMeterSecond, ampsPerMeterSecond);

    // Kinematic / Chassis
    assertNotNull(MetersPerSecondPerMeter);
    assertSame(MetersPerSecondPerMeter, metersPerSecondPerMeter);
    assertNotNull(MetersPerSecondPerMeterSecond);
    assertSame(MetersPerSecondPerMeterSecond, metersPerSecondPerMeterSecond);
    assertNotNull(MetersPerSecondPerMetersPerSecond);
    assertSame(MetersPerSecondPerMetersPerSecond, metersPerSecondPerMetersPerSecond);
    assertNotNull(RadiansPerSecondPerRadian);
    assertSame(RadiansPerSecondPerRadian, radiansPerSecondPerRadian);
    assertNotNull(DegreesPerSecondPerDegree);
    assertSame(DegreesPerSecondPerDegree, degreesPerSecondPerDegree);
    assertNotNull(RadiansPerSecondPerRadianSecond);
    assertSame(RadiansPerSecondPerRadianSecond, radiansPerSecondPerRadianSecond);
    assertNotNull(RadiansPerSecondPerRadiansPerSecond);
    assertSame(RadiansPerSecondPerRadiansPerSecond, radiansPerSecondPerRadiansPerSecond);
    assertNotNull(DegreesPerSecondPerDegreesPerSecond);
    assertSame(DegreesPerSecondPerDegreesPerSecond, degreesPerSecondPerDegreesPerSecond);

    // Motor Characteristics
    assertNotNull(NewtonMetersPerAmp);
    assertSame(NewtonMetersPerAmp, newtonMetersPerAmp);
    assertNotNull(RadiansPerSecondPerVolt);
    assertSame(RadiansPerSecondPerVolt, radiansPerSecondPerVolt);
    assertNotNull(RotationsPerSecondPerVolt);
    assertSame(RotationsPerSecondPerVolt, rotationsPerSecondPerVolt);
  }
}
