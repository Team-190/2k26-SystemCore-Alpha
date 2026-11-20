package edu.wpi.team190.gompeilib.io.gyro.config;

import edu.wpi.team190.gompeilib.io.base.Signal;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import edu.wpi.team190.gompeilib.io.gyro.inputs.GyroTelemetry;

/** The status signals {@link GyroTelemetry} is filled from, each in one {@link GyroSignalGroup}. */
public enum GyroSignal implements Signal<GyroSignalGroup> {
  /** {@link GyroTelemetry#yawPosition}. */
  YAW_POSITION(GyroSignalGroup.CONTROL),

  /** {@link GyroTelemetry#pitchPosition}. */
  PITCH_POSITION(GyroSignalGroup.CONTROL),

  /** {@link GyroTelemetry#rollPosition}. */
  ROLL_POSITION(GyroSignalGroup.CONTROL),

  /** {@link GyroTelemetry#yawVelocity}. */
  YAW_VELOCITY(GyroSignalGroup.CONTROL),

  /** {@link GyroTelemetry#pitchVelocity}. */
  PITCH_VELOCITY(GyroSignalGroup.CONTROL),

  /** {@link GyroTelemetry#rollVelocity}. */
  ROLL_VELOCITY(GyroSignalGroup.CONTROL),

  /** {@link GyroTelemetry#isProLicensed}. */
  IS_PRO_LICENSED(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#firmwareVersion}. */
  FIRMWARE_VERSION(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#deviceAngularVelocityX}. */
  DEVICE_ANGULAR_VELOCITY_X(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#deviceAngularVelocityY}. */
  DEVICE_ANGULAR_VELOCITY_Y(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#deviceAngularVelocityZ}. */
  DEVICE_ANGULAR_VELOCITY_Z(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#accelerationX}. */
  ACCELERATION_X(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#accelerationY}. */
  ACCELERATION_Y(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#accelerationZ}. */
  ACCELERATION_Z(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#quaternionW}. */
  QUATERNION_W(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#quaternionX}. */
  QUATERNION_X(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#quaternionY}. */
  QUATERNION_Y(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#quaternionZ}. */
  QUATERNION_Z(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#gravityVectorX}. */
  GRAVITY_VECTOR_X(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#gravityVectorY}. */
  GRAVITY_VECTOR_Y(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#gravityVectorZ}. */
  GRAVITY_VECTOR_Z(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#accumulatedRotationX}. */
  ACCUMULATED_ROTATION_X(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#accumulatedRotationY}. */
  ACCUMULATED_ROTATION_Y(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#accumulatedRotationZ}. */
  ACCUMULATED_ROTATION_Z(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#magneticFieldX}. */
  MAGNETIC_FIELD_X(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#magneticFieldY}. */
  MAGNETIC_FIELD_Y(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#magneticFieldZ}. */
  MAGNETIC_FIELD_Z(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#rawMagneticFieldX}. */
  RAW_MAGNETIC_FIELD_X(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#rawMagneticFieldY}. */
  RAW_MAGNETIC_FIELD_Y(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#rawMagneticFieldZ}. */
  RAW_MAGNETIC_FIELD_Z(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#temperature}. */
  TEMPERATURE(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#temperatureCompensationDisabled}. */
  TEMPERATURE_COMPENSATION_DISABLED(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#noMotionEnabled}. */
  NO_MOTION_ENABLED(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#noMotionCount}. */
  NO_MOTION_COUNT(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#upTime}. */
  UP_TIME(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#supplyVoltage}. */
  SUPPLY_VOLTAGE(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#faults}. */
  FAULTS(GyroSignalGroup.DIAGNOSTIC),

  /** {@link GyroTelemetry#stickyFaults}. */
  STICKY_FAULTS(GyroSignalGroup.DIAGNOSTIC);

  /** Every group at its default rate, with no signal overrides. */
  public static final SignalRates<GyroSignal, GyroSignalGroup> DEFAULT_RATES =
      SignalRates.defaults(GyroSignal.class, GyroSignalGroup.class);

  private final GyroSignalGroup group;

  GyroSignal(GyroSignalGroup group) {
    this.group = group;
  }

  @Override
  public GyroSignalGroup group() {
    return group;
  }
}
