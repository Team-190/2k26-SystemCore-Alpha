package edu.wpi.team190.gompeilib.core.io.gyro;

import static java.util.Map.entry;

import com.ctre.phoenix6.hardware.core.CorePigeon2;
import edu.wpi.team190.gompeilib.core.io.IOTestSupport;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

public class GyroTelemetryTest {

  /** Which {@link GyroTelemetry} fields each Pigeon2 status signal fills. */
  public static final Map<String, List<String>> PIGEON2_SIGNAL_TO_FIELDS =
      Map.ofEntries(
          entry("Version", List.of("firmwareVersion")),
          entry("IsProLicensed", List.of("isProLicensed")),
          entry("Yaw", List.of("yawPosition")),
          entry("Pitch", List.of("pitchPosition")),
          entry("Roll", List.of("rollPosition")),
          // Phoenix6 defines roll about X, pitch about Y, and yaw about Z.
          entry("AngularVelocityXWorld", List.of("rollVelocity")),
          entry("AngularVelocityYWorld", List.of("pitchVelocity")),
          entry("AngularVelocityZWorld", List.of("yawVelocity")),
          entry("AngularVelocityXDevice", List.of("deviceAngularVelocityX")),
          entry("AngularVelocityYDevice", List.of("deviceAngularVelocityY")),
          entry("AngularVelocityZDevice", List.of("deviceAngularVelocityZ")),
          entry("AccelerationX", List.of("accelerationX")),
          entry("AccelerationY", List.of("accelerationY")),
          entry("AccelerationZ", List.of("accelerationZ")),
          entry("QuatW", List.of("quaternionW")),
          entry("QuatX", List.of("quaternionX")),
          entry("QuatY", List.of("quaternionY")),
          entry("QuatZ", List.of("quaternionZ")),
          entry("GravityVectorX", List.of("gravityVectorX")),
          entry("GravityVectorY", List.of("gravityVectorY")),
          entry("GravityVectorZ", List.of("gravityVectorZ")),
          entry("AccumGyroX", List.of("accumulatedRotationX")),
          entry("AccumGyroY", List.of("accumulatedRotationY")),
          entry("AccumGyroZ", List.of("accumulatedRotationZ")),
          entry("MagneticFieldX", List.of("magneticFieldX")),
          entry("MagneticFieldY", List.of("magneticFieldY")),
          entry("MagneticFieldZ", List.of("magneticFieldZ")),
          entry("RawMagneticFieldX", List.of("rawMagneticFieldX")),
          entry("RawMagneticFieldY", List.of("rawMagneticFieldY")),
          entry("RawMagneticFieldZ", List.of("rawMagneticFieldZ")),
          entry("Temperature", List.of("temperature")),
          entry("TemperatureCompensationDisabled", List.of("temperatureCompensationDisabled")),
          entry("NoMotionEnabled", List.of("noMotionEnabled")),
          entry("NoMotionCount", List.of("noMotionCount")),
          entry("UpTime", List.of("upTime")),
          entry("SupplyVoltage", List.of("supplyVoltage")));

  /** {@link GyroTelemetry} fields no single Pigeon2 status signal fills, and why. */
  public static final Map<String, String> PIGEON2_FIELDS_WITHOUT_SIGNAL =
      IOTestSupport.with(
          IOTestSupport.COMMON_FIELDS_WITHOUT_SIGNAL,
          Map.of(
              "isCalibrating", "Pigeon2 publishes no calibration state",
              "odometryTimestamps", "filled from the high-frequency odometry thread",
              "odometryYawPositions", "filled from the high-frequency odometry thread"));

  @Test
  public void testDefaultValues() throws IllegalAccessException {
    IOTestSupport.assertZeroDefaults(new GyroTelemetry());
  }

  @Test
  public void testCoversEveryPigeon2Signal() {
    IOTestSupport.assertTelemetryCoverage(
        CorePigeon2.class,
        GyroTelemetry.class,
        PIGEON2_SIGNAL_TO_FIELDS,
        IOTestSupport.COMMON_UNMAPPED_SIGNALS,
        PIGEON2_FIELDS_WITHOUT_SIGNAL);
  }
}
