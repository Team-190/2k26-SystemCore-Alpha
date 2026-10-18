package edu.wpi.team190.gompeilib.core.io.range;

import static org.wpilib.units.Units.*;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.*;

/**
 * Everything a range (time-of-flight) sensor reports. Fields a device doesn't report stay at their
 * defaults.
 */
@AutoLog
public class RangeSensorTelemetry {
  /** Whether the device is connected and communicating. */
  public boolean connected = false;

  /** Whether the device has a Pro license. */
  public boolean isProLicensed = false;

  /** The device's packed firmware version. */
  public int firmwareVersion = 0;

  /** Measured distance to the target. */
  public Distance distance = Meters.of(0.0);

  /** Standard deviation of the distance measurement. */
  public Distance distanceStdDev = Meters.of(0.0);

  /** Time the measurement took. */
  public Time measurementTime = Seconds.of(0.0);

  /** Whether a target is detected within the configured thresholds. */
  public boolean detected = false;

  /** Strength of the returned signal, in the vendor's unitless scale. */
  public double signalStrength = 0.0;

  /** Strength of the ambient (non-returned) signal, in the vendor's unitless scale. */
  public double ambientSignal = 0.0;

  /** How trustworthy the measurement is. */
  public MeasurementHealth measurementHealth = MeasurementHealth.UNKNOWN;

  /** Center of the field of view along X. */
  public Angle fovCenterX = Radians.of(0.0);

  /** Center of the field of view along Y. */
  public Angle fovCenterY = Radians.of(0.0);

  /** Width of the field of view along X. */
  public Angle fovRangeX = Radians.of(0.0);

  /** Width of the field of view along Y. */
  public Angle fovRangeY = Radians.of(0.0);

  /** Voltage supplied to the device. */
  public Voltage supplyVoltage = Volts.of(0.0);

  /** Live faults, packed per the vendor's fault enum. */
  public long faults = 0L;

  /** Sticky faults, packed per the vendor's fault enum. */
  public long stickyFaults = 0L;
}
