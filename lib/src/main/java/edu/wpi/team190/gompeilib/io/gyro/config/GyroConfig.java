package edu.wpi.team190.gompeilib.io.gyro.config;

import static org.wpilib.units.Units.Degrees;

import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import lombok.Builder;
import lombok.NonNull;
import org.wpilib.units.measure.Angle;

/** Everything one gyro needs. */
@Builder(setterPrefix = "with", toBuilder = true)
public class GyroConfig {
  /** The gyro's CAN ID. */
  @NonNull public final Integer canId;

  /** The bus the gyro is on. */
  @NonNull public final CANBusId bus;

  /** The gyro's mounting yaw relative to the robot. */
  @NonNull @Builder.Default public final Angle mountPoseYaw = Degrees.of(0.0);

  /** The gyro's mounting pitch relative to the robot. */
  @NonNull @Builder.Default public final Angle mountPosePitch = Degrees.of(0.0);

  /** The gyro's mounting roll relative to the robot. */
  @NonNull @Builder.Default public final Angle mountPoseRoll = Degrees.of(0.0);

  /** Whether to fuse the magnetometer into the yaw estimate. */
  @NonNull @Builder.Default public final Boolean enableCompass = false;

  /** Whether to disable temperature compensation. */
  @NonNull @Builder.Default public final Boolean disableTemperatureCompensation = false;

  /** Whether to disable automatic bias calibration while still. */
  @NonNull @Builder.Default public final Boolean disableNoMotionCalibration = false;

  /** How often each telemetry field is published. */
  @NonNull @Builder.Default
  public final SignalRates<GyroSignal, GyroSignalGroup> signalRates = GyroSignal.DEFAULT_RATES;
}
