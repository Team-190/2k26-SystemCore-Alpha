package edu.wpi.team190.gompeilib.io.encoder.inputs;

import static org.wpilib.units.Units.*;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.*;

/**
 * Everything an absolute encoder reports. Fields a device doesn't report stay at their defaults.
 */
@AutoLog
public class AbsoluteEncoderTelemetry {
  /** Whether the device is connected and communicating. */
  public boolean connected = false;

  /** Whether the device has a Pro license. */
  public boolean isProLicensed = false;

  /** The device's packed firmware version. */
  public int firmwareVersion = 0;

  /** Absolute position within one rotation, after the configured offset. */
  public Angle absolutePosition = Radians.of(0.0);

  /** Continuous position, including full rotations. */
  public Angle position = Radians.of(0.0);

  /** Continuous position since the device booted, unaffected by position resets. */
  public Angle positionSinceBoot = Radians.of(0.0);

  /** Filtered velocity. */
  public AngularVelocity velocity = RadiansPerSecond.of(0.0);

  /** Unfiltered velocity. */
  public AngularVelocity unfilteredVelocity = RadiansPerSecond.of(0.0);

  /** Health of the magnet the encoder reads. */
  public MagnetHealth magnetHealth = MagnetHealth.UNKNOWN;

  /** Voltage supplied to the device. */
  public Voltage supplyVoltage = Volts.of(0.0);

  /** Live faults, packed per the vendor's fault enum. */
  public long faults = 0L;

  /** Sticky faults, packed per the vendor's fault enum. */
  public long stickyFaults = 0L;
}
