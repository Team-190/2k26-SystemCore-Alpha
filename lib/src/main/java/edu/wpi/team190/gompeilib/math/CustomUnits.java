package edu.wpi.team190.gompeilib.math;

import static org.wpilib.units.Units.*;

import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularAccelerationUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.DimensionlessUnit;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.LinearAccelerationUnit;
import org.wpilib.units.LinearVelocityUnit;
import org.wpilib.units.MultUnit;
import org.wpilib.units.PerUnit;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.TorqueUnit;
import org.wpilib.units.VelocityUnit;
import org.wpilib.units.VoltageUnit;

public final class CustomUnits {
  private CustomUnits() {}

  // Ramp Rates & Rates of Change
  public static final PerUnit<CurrentUnit, TimeUnit> AmpsPerSecond = Amps.per(Second);
  public static final PerUnit<VoltageUnit, TimeUnit> VoltsPerSecond = Volts.per(Second);
  public static final PerUnit<DimensionlessUnit, TimeUnit> PercentPerSecond = Percent.per(Second);

  // Jerk / Acceleration Rates
  public static final VelocityUnit<AngularAccelerationUnit> RotationsPerSecondCubed =
      RotationsPerSecondPerSecond.per(Second);
  public static final VelocityUnit<AngularAccelerationUnit> RadiansPerSecondCubed =
      RadiansPerSecondPerSecond.per(Second);
  public static final VelocityUnit<AngularAccelerationUnit> DegreesPerSecondCubed =
      DegreesPerSecondPerSecond.per(Second);
  public static final VelocityUnit<LinearAccelerationUnit> MetersPerSecondCubed =
      MetersPerSecondPerSecond.per(Second);

  // Angular Feedback & Feedforward (Duty Cycle / Percent Output)
  public static final PerUnit<DimensionlessUnit, AngleUnit> PercentPerRadian = Percent.per(Radian);
  public static final PerUnit<DimensionlessUnit, AngleUnit> PercentPerRotation =
      Percent.per(Rotation);
  public static final PerUnit<DimensionlessUnit, AngleUnit> PercentPerDegree = Percent.per(Degree);

  // Angular Feedback & Feedforward (Voltage Output)
  public static final PerUnit<VoltageUnit, AngleUnit> VoltsPerRadian = Volts.per(Radian);
  public static final PerUnit<VoltageUnit, AngleUnit> VoltsPerRotation = Volts.per(Rotation);
  public static final PerUnit<VoltageUnit, AngleUnit> VoltsPerDegree = Volts.per(Degree);

  public static final PerUnit<VoltageUnit, AngularVelocityUnit> VoltsPerRotationPerSecond =
      Volts.per(RotationsPerSecond);
  public static final PerUnit<VoltageUnit, AngularVelocityUnit> VoltsPerDegreePerSecond =
      Volts.per(DegreesPerSecond);

  public static final PerUnit<VoltageUnit, AngularAccelerationUnit>
      VoltsPerRotationPerSecondSquared = Volts.per(RotationsPerSecondPerSecond);
  public static final PerUnit<VoltageUnit, AngularAccelerationUnit> VoltsPerDegreePerSecondSquared =
      Volts.per(DegreesPerSecondPerSecond);

  public static final PerUnit<VoltageUnit, MultUnit<AngleUnit, TimeUnit>> VoltsPerRadianSecond =
      Volts.per(MultUnit.combine(Radians, Seconds));
  public static final PerUnit<VoltageUnit, MultUnit<AngleUnit, TimeUnit>> VoltsPerRotationSecond =
      Volts.per(MultUnit.combine(Rotations, Seconds));

  // Angular Feedback & Feedforward (Current Output)
  public static final PerUnit<CurrentUnit, AngleUnit> AmpsPerRadian = Amps.per(Radian);
  public static final PerUnit<CurrentUnit, AngleUnit> AmpsPerRotation = Amps.per(Rotation);
  public static final PerUnit<CurrentUnit, AngleUnit> AmpsPerDegree = Amps.per(Degree);

  public static final PerUnit<CurrentUnit, AngularVelocityUnit> AmpsPerRadianPerSecond =
      Amps.per(RadiansPerSecond);
  public static final PerUnit<CurrentUnit, AngularVelocityUnit> AmpsPerRotationPerSecond =
      Amps.per(RotationsPerSecond);
  public static final PerUnit<CurrentUnit, AngularVelocityUnit> AmpsPerDegreePerSecond =
      Amps.per(DegreesPerSecond);

  public static final PerUnit<CurrentUnit, AngularAccelerationUnit> AmpsPerRadianPerSecondSquared =
      Amps.per(RadiansPerSecondPerSecond);
  public static final PerUnit<CurrentUnit, AngularAccelerationUnit>
      AmpsPerRotationPerSecondSquared = Amps.per(RotationsPerSecondPerSecond);
  public static final PerUnit<CurrentUnit, AngularAccelerationUnit> AmpsPerDegreePerSecondSquared =
      Amps.per(DegreesPerSecondPerSecond);

  public static final PerUnit<CurrentUnit, MultUnit<AngleUnit, TimeUnit>> AmpsPerRadianSecond =
      Amps.per(MultUnit.combine(Radians, Seconds));
  public static final PerUnit<CurrentUnit, MultUnit<AngleUnit, TimeUnit>> AmpsPerRotationSecond =
      Amps.per(MultUnit.combine(Rotations, Seconds));

  // Linear Feedback & Feedforward (Voltage Output)
  public static final PerUnit<VoltageUnit, DistanceUnit> VoltsPerMeter = Volts.per(Meter);
  public static final PerUnit<VoltageUnit, DistanceUnit> VoltsPerInch = Volts.per(Inches);

  public static final PerUnit<VoltageUnit, LinearVelocityUnit> VoltsPerInchPerSecond =
      Volts.per(Inches.per(Second));

  public static final PerUnit<VoltageUnit, MultUnit<DistanceUnit, TimeUnit>> VoltsPerMeterSecond =
      Volts.per(MultUnit.combine(Meters, Seconds));

  // Linear Feedback & Feedforward (Current Output)
  public static final PerUnit<CurrentUnit, DistanceUnit> AmpsPerMeter = Amps.per(Meter);
  public static final PerUnit<CurrentUnit, DistanceUnit> AmpsPerInch = Amps.per(Inches);

  public static final PerUnit<CurrentUnit, LinearVelocityUnit> AmpsPerMeterPerSecond =
      Amps.per(MetersPerSecond);
  public static final PerUnit<CurrentUnit, LinearVelocityUnit> AmpsPerInchPerSecond =
      Amps.per(Inches.per(Second));

  public static final PerUnit<CurrentUnit, LinearAccelerationUnit> AmpsPerMeterPerSecondSquared =
      Amps.per(MetersPerSecondPerSecond);

  public static final PerUnit<CurrentUnit, MultUnit<DistanceUnit, TimeUnit>> AmpsPerMeterSecond =
      Amps.per(MultUnit.combine(Meters, Seconds));

  // Kinematic / Chassis Gains (Velocity Output)
  public static final PerUnit<LinearVelocityUnit, DistanceUnit> MetersPerSecondPerMeter =
      MetersPerSecond.per(Meter);
  public static final PerUnit<LinearVelocityUnit, MultUnit<DistanceUnit, TimeUnit>>
      MetersPerSecondPerMeterSecond = MetersPerSecond.per(MultUnit.combine(Meters, Seconds));
  public static final PerUnit<LinearVelocityUnit, LinearVelocityUnit>
      MetersPerSecondPerMetersPerSecond = MetersPerSecond.per(MetersPerSecond);

  public static final PerUnit<AngularVelocityUnit, AngleUnit> RadiansPerSecondPerRadian =
      RadiansPerSecond.per(Radian);
  public static final PerUnit<AngularVelocityUnit, AngleUnit> DegreesPerSecondPerDegree =
      DegreesPerSecond.per(Degree);
  public static final PerUnit<AngularVelocityUnit, MultUnit<AngleUnit, TimeUnit>>
      RadiansPerSecondPerRadianSecond = RadiansPerSecond.per(MultUnit.combine(Radians, Seconds));
  public static final PerUnit<AngularVelocityUnit, AngularVelocityUnit>
      RadiansPerSecondPerRadiansPerSecond = RadiansPerSecond.per(RadiansPerSecond);
  public static final PerUnit<AngularVelocityUnit, AngularVelocityUnit>
      DegreesPerSecondPerDegreesPerSecond = DegreesPerSecond.per(DegreesPerSecond);

  // Motor Characteristics
  public static final PerUnit<TorqueUnit, CurrentUnit> NewtonMetersPerAmp = NewtonMeters.per(Amp);
  public static final PerUnit<AngularVelocityUnit, VoltageUnit> RadiansPerSecondPerVolt =
      RadiansPerSecond.per(Volt);
  public static final PerUnit<AngularVelocityUnit, VoltageUnit> RotationsPerSecondPerVolt =
      RotationsPerSecond.per(Volt);

  // Aliases for backwards compatibility
  public static final PerUnit<CurrentUnit, TimeUnit> ampsPerSecond = AmpsPerSecond;
  public static final PerUnit<VoltageUnit, TimeUnit> voltsPerSecond = VoltsPerSecond;
  public static final PerUnit<DimensionlessUnit, TimeUnit> percentPerSecond = PercentPerSecond;

  public static final VelocityUnit<AngularAccelerationUnit> rotationsPerSecondCubed =
      RotationsPerSecondCubed;
  public static final VelocityUnit<AngularAccelerationUnit> radiansPerSecondCubed =
      RadiansPerSecondCubed;
  public static final VelocityUnit<AngularAccelerationUnit> degreesPerSecondCubed =
      DegreesPerSecondCubed;
  public static final VelocityUnit<LinearAccelerationUnit> metersPerSecondCubed =
      MetersPerSecondCubed;

  public static final PerUnit<DimensionlessUnit, AngleUnit> percentPerRadian = PercentPerRadian;
  public static final PerUnit<DimensionlessUnit, AngleUnit> percentPerRotation = PercentPerRotation;
  public static final PerUnit<DimensionlessUnit, AngleUnit> percentPerDegree = PercentPerDegree;

  public static final PerUnit<VoltageUnit, AngleUnit> voltsPerRadian = VoltsPerRadian;
  public static final PerUnit<VoltageUnit, AngleUnit> voltsPerRotation = VoltsPerRotation;
  public static final PerUnit<VoltageUnit, AngleUnit> voltsPerDegree = VoltsPerDegree;
  public static final PerUnit<VoltageUnit, AngularVelocityUnit> voltsPerRadianPerSecond =
      VoltsPerRadianPerSecond;
  public static final PerUnit<VoltageUnit, AngularVelocityUnit> voltsPerRotationPerSecond =
      VoltsPerRotationPerSecond;
  public static final PerUnit<VoltageUnit, AngularVelocityUnit> voltsPerDegreePerSecond =
      VoltsPerDegreePerSecond;
  public static final PerUnit<VoltageUnit, AngularAccelerationUnit> voltsPerRadianPerSecondSquared =
      VoltsPerRadianPerSecondSquared;
  public static final PerUnit<VoltageUnit, AngularAccelerationUnit>
      voltsPerRotationPerSecondSquared = VoltsPerRotationPerSecondSquared;
  public static final PerUnit<VoltageUnit, AngularAccelerationUnit> voltsPerDegreePerSecondSquared =
      VoltsPerDegreePerSecondSquared;
  public static final PerUnit<VoltageUnit, MultUnit<AngleUnit, TimeUnit>> voltsPerRadianSecond =
      VoltsPerRadianSecond;
  public static final PerUnit<VoltageUnit, MultUnit<AngleUnit, TimeUnit>> voltsPerRotationSecond =
      VoltsPerRotationSecond;

  public static final PerUnit<CurrentUnit, AngleUnit> ampsPerRadian = AmpsPerRadian;
  public static final PerUnit<CurrentUnit, AngleUnit> ampsPerRotation = AmpsPerRotation;
  public static final PerUnit<CurrentUnit, AngleUnit> ampsPerDegree = AmpsPerDegree;
  public static final PerUnit<CurrentUnit, AngularVelocityUnit> ampsPerRadianPerSecond =
      AmpsPerRadianPerSecond;
  public static final PerUnit<CurrentUnit, AngularVelocityUnit> ampsPerRotationPerSecond =
      AmpsPerRotationPerSecond;
  public static final PerUnit<CurrentUnit, AngularVelocityUnit> ampsPerDegreePerSecond =
      AmpsPerDegreePerSecond;
  public static final PerUnit<CurrentUnit, AngularAccelerationUnit> ampsPerRadianPerSecondSquared =
      AmpsPerRadianPerSecondSquared;
  public static final PerUnit<CurrentUnit, AngularAccelerationUnit>
      ampsPerRotationPerSecondSquared = AmpsPerRotationPerSecondSquared;
  public static final PerUnit<CurrentUnit, AngularAccelerationUnit> ampsPerDegreePerSecondSquared =
      AmpsPerDegreePerSecondSquared;
  public static final PerUnit<CurrentUnit, MultUnit<AngleUnit, TimeUnit>> ampsPerRadianSecond =
      AmpsPerRadianSecond;
  public static final PerUnit<CurrentUnit, MultUnit<AngleUnit, TimeUnit>> ampsPerRotationSecond =
      AmpsPerRotationSecond;

  public static final PerUnit<VoltageUnit, DistanceUnit> voltsPerMeter = VoltsPerMeter;
  public static final PerUnit<VoltageUnit, DistanceUnit> voltsPerInch = VoltsPerInch;
  public static final PerUnit<VoltageUnit, LinearVelocityUnit> voltsPerMeterPerSecond =
      VoltsPerMeterPerSecond;
  public static final PerUnit<VoltageUnit, LinearVelocityUnit> voltsPerInchPerSecond =
      VoltsPerInchPerSecond;
  public static final PerUnit<VoltageUnit, LinearAccelerationUnit> voltsPerMeterPerSecondSquared =
      VoltsPerMeterPerSecondSquared;
  public static final PerUnit<VoltageUnit, MultUnit<DistanceUnit, TimeUnit>> voltsPerMeterSecond =
      VoltsPerMeterSecond;

  public static final PerUnit<CurrentUnit, DistanceUnit> ampsPerMeter = AmpsPerMeter;
  public static final PerUnit<CurrentUnit, DistanceUnit> ampsPerInch = AmpsPerInch;
  public static final PerUnit<CurrentUnit, LinearVelocityUnit> ampsPerMeterPerSecond =
      AmpsPerMeterPerSecond;
  public static final PerUnit<CurrentUnit, LinearVelocityUnit> ampsPerInchPerSecond =
      AmpsPerInchPerSecond;
  public static final PerUnit<CurrentUnit, LinearAccelerationUnit> ampsPerMeterPerSecondSquared =
      AmpsPerMeterPerSecondSquared;
  public static final PerUnit<CurrentUnit, MultUnit<DistanceUnit, TimeUnit>> ampsPerMeterSecond =
      AmpsPerMeterSecond;

  public static final PerUnit<LinearVelocityUnit, DistanceUnit> metersPerSecondPerMeter =
      MetersPerSecondPerMeter;
  public static final PerUnit<LinearVelocityUnit, MultUnit<DistanceUnit, TimeUnit>>
      metersPerSecondPerMeterSecond = MetersPerSecondPerMeterSecond;
  public static final PerUnit<LinearVelocityUnit, LinearVelocityUnit>
      metersPerSecondPerMetersPerSecond = MetersPerSecondPerMetersPerSecond;

  public static final PerUnit<AngularVelocityUnit, AngleUnit> radiansPerSecondPerRadian =
      RadiansPerSecondPerRadian;
  public static final PerUnit<AngularVelocityUnit, AngleUnit> degreesPerSecondPerDegree =
      DegreesPerSecondPerDegree;
  public static final PerUnit<AngularVelocityUnit, MultUnit<AngleUnit, TimeUnit>>
      radiansPerSecondPerRadianSecond = RadiansPerSecondPerRadianSecond;
  public static final PerUnit<AngularVelocityUnit, AngularVelocityUnit>
      radiansPerSecondPerRadiansPerSecond = RadiansPerSecondPerRadiansPerSecond;
  public static final PerUnit<AngularVelocityUnit, AngularVelocityUnit>
      degreesPerSecondPerDegreesPerSecond = DegreesPerSecondPerDegreesPerSecond;

  public static final PerUnit<TorqueUnit, CurrentUnit> newtonMetersPerAmp = NewtonMetersPerAmp;
  public static final PerUnit<AngularVelocityUnit, VoltageUnit> radiansPerSecondPerVolt =
      RadiansPerSecondPerVolt;
  public static final PerUnit<AngularVelocityUnit, VoltageUnit> rotationsPerSecondPerVolt =
      RotationsPerSecondPerVolt;
}
