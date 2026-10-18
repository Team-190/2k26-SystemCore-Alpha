package edu.wpi.team190.gompeilib.io.range.phoenix;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.signals.MeasurementHealthValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.team190.gompeilib.io.range.config.RangeSensorConfig;
import edu.wpi.team190.gompeilib.io.range.inputs.MeasurementHealth;

/** Pure conversions between GompeiLib's range sensor types and Phoenix6's CANrange types. */
public final class CANrangeConversions {
  private CANrangeConversions() {}

  /**
   * Builds the device configuration for a range sensor.
   *
   * @param config the sensor's config
   * @return the CANrange configuration
   */
  public static CANrangeConfiguration toConfiguration(RangeSensorConfig config) {
    CANrangeConfiguration canrange = new CANrangeConfiguration();
    canrange.ProximityParams.ProximityThreshold = config.proximityThreshold.in(Meters);
    canrange.ProximityParams.ProximityHysteresis = config.proximityHysteresis.in(Meters);
    canrange.ProximityParams.MinSignalStrengthForValidMeasurement = config.minSignalStrength;
    canrange.ToFParams.UpdateMode =
        switch (config.updateMode) {
          case SHORT_RANGE_100HZ -> UpdateModeValue.ShortRange100Hz;
          case SHORT_RANGE_USER_FREQUENCY -> UpdateModeValue.ShortRangeUserFreq;
          case LONG_RANGE_USER_FREQUENCY -> UpdateModeValue.LongRangeUserFreq;
        };
    canrange.ToFParams.UpdateFrequency = config.updateFrequency.in(Hertz);
    canrange.FovParams.FOVCenterX = config.fovCenterX.in(Degrees);
    canrange.FovParams.FOVCenterY = config.fovCenterY.in(Degrees);
    canrange.FovParams.FOVRangeX = config.fovRangeX.in(Degrees);
    canrange.FovParams.FOVRangeY = config.fovRangeY.in(Degrees);
    return canrange;
  }

  /**
   * Converts a reported measurement health.
   *
   * @param health the Phoenix6 measurement health
   * @return the measurement health
   */
  public static MeasurementHealth toMeasurementHealth(MeasurementHealthValue health) {
    return switch (health) {
      case Good -> MeasurementHealth.GOOD;
      case Limited -> MeasurementHealth.LIMITED;
      case Bad -> MeasurementHealth.BAD;
    };
  }
}
