package edu.wpi.team190.gompeilib.core.io.gyro;

import static org.wpilib.units.Units.*;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.*;

/**
 * Everything a gyro reports. Roll is about X, pitch about Y, and yaw about Z. Fields a device
 * doesn't report stay at their defaults.
 */
@AutoLog
public class GyroTelemetry {
  /** Whether the device is connected and communicating. */
  public boolean connected = false;

  /** Whether the device has a Pro license. */
  public boolean isProLicensed = false;

  /** The device's packed firmware version. */
  public int firmwareVersion = 0;

  /** Yaw (rotation about Z). */
  public Angle yawPosition = Radians.of(0.0);

  /** Pitch (rotation about Y). */
  public Angle pitchPosition = Radians.of(0.0);

  /** Roll (rotation about X). */
  public Angle rollPosition = Radians.of(0.0);

  /** Yaw rate, in the world frame. */
  public AngularVelocity yawVelocity = RadiansPerSecond.of(0.0);

  /** Pitch rate, in the world frame. */
  public AngularVelocity pitchVelocity = RadiansPerSecond.of(0.0);

  /** Roll rate, in the world frame. */
  public AngularVelocity rollVelocity = RadiansPerSecond.of(0.0);

  /** Angular velocity about the device's own X axis. */
  public AngularVelocity deviceAngularVelocityX = RadiansPerSecond.of(0.0);

  /** Angular velocity about the device's own Y axis. */
  public AngularVelocity deviceAngularVelocityY = RadiansPerSecond.of(0.0);

  /** Angular velocity about the device's own Z axis. */
  public AngularVelocity deviceAngularVelocityZ = RadiansPerSecond.of(0.0);

  /** Raw acceleration along X. */
  public LinearAcceleration accelerationX = MetersPerSecondPerSecond.of(0.0);

  /** Raw acceleration along Y. */
  public LinearAcceleration accelerationY = MetersPerSecondPerSecond.of(0.0);

  /** Raw acceleration along Z. */
  public LinearAcceleration accelerationZ = MetersPerSecondPerSecond.of(0.0);

  /** Orientation quaternion, W component. */
  public double quaternionW = 0.0;

  /** Orientation quaternion, X component. */
  public double quaternionX = 0.0;

  /** Orientation quaternion, Y component. */
  public double quaternionY = 0.0;

  /** Orientation quaternion, Z component. */
  public double quaternionZ = 0.0;

  /** Unit gravity vector, X component. */
  public double gravityVectorX = 0.0;

  /** Unit gravity vector, Y component. */
  public double gravityVectorY = 0.0;

  /** Unit gravity vector, Z component. */
  public double gravityVectorZ = 0.0;

  /** Total rotation accumulated about X since boot. */
  public Angle accumulatedRotationX = Radians.of(0.0);

  /** Total rotation accumulated about Y since boot. */
  public Angle accumulatedRotationY = Radians.of(0.0);

  /** Total rotation accumulated about Z since boot. */
  public Angle accumulatedRotationZ = Radians.of(0.0);

  /** Biased magnetic field strength along X, in microteslas. */
  public double magneticFieldX = 0.0;

  /** Biased magnetic field strength along Y, in microteslas. */
  public double magneticFieldY = 0.0;

  /** Biased magnetic field strength along Z, in microteslas. */
  public double magneticFieldZ = 0.0;

  /** Raw magnetic field strength along X, in microteslas. */
  public double rawMagneticFieldX = 0.0;

  /** Raw magnetic field strength along Y, in microteslas. */
  public double rawMagneticFieldY = 0.0;

  /** Raw magnetic field strength along Z, in microteslas. */
  public double rawMagneticFieldZ = 0.0;

  /** Device temperature. */
  public Temperature temperature = Celsius.of(0.0);

  /** Whether temperature compensation is disabled. */
  public boolean temperatureCompensationDisabled = false;

  /** Whether automatic no-motion bias calibration is enabled. */
  public boolean noMotionEnabled = false;

  /** Number of times no-motion calibration has run. */
  public double noMotionCount = 0.0;

  /** Whether the device is still calibrating. Stays false on devices that don't report it. */
  public boolean isCalibrating = false;

  /** Time since the device booted. */
  public Time upTime = Seconds.of(0.0);

  /** Voltage supplied to the device. */
  public Voltage supplyVoltage = Volts.of(0.0);

  /** Live faults, packed per the vendor's fault enum. */
  public long faults = 0L;

  /** Sticky faults, packed per the vendor's fault enum. */
  public long stickyFaults = 0L;

  /** Timestamps of the yaw samples captured between main-loop cycles, for odometry. */
  public double[] odometryTimestamps = new double[] {};

  /** Yaw samples captured between main-loop cycles, for odometry. */
  public Angle[] odometryYawPositions = new Angle[] {};
}
