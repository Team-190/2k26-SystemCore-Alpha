package edu.wpi.team190.gompeilib.core.io.range;

import static java.util.Map.entry;

import com.ctre.phoenix6.hardware.core.CoreCANrange;
import edu.wpi.team190.gompeilib.core.io.IOTestSupport;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

public class RangeSensorTelemetryTest {

  /** Which {@link RangeSensorTelemetry} fields each CANrange status signal fills. */
  public static final Map<String, List<String>> CANRANGE_SIGNAL_TO_FIELDS =
      Map.ofEntries(
          entry("Version", List.of("firmwareVersion")),
          entry("IsProLicensed", List.of("isProLicensed")),
          entry("Distance", List.of("distance")),
          entry("DistanceStdDev", List.of("distanceStdDev")),
          entry("MeasurementTime", List.of("measurementTime")),
          entry("IsDetected", List.of("detected")),
          entry("SignalStrength", List.of("signalStrength")),
          entry("AmbientSignal", List.of("ambientSignal")),
          entry("MeasurementHealth", List.of("measurementHealth")),
          entry("RealFOVCenterX", List.of("fovCenterX")),
          entry("RealFOVCenterY", List.of("fovCenterY")),
          entry("RealFOVRangeX", List.of("fovRangeX")),
          entry("RealFOVRangeY", List.of("fovRangeY")),
          entry("SupplyVoltage", List.of("supplyVoltage")));

  @Test
  public void testDefaultValues() throws IllegalAccessException {
    IOTestSupport.assertZeroDefaults(new RangeSensorTelemetry());
  }

  @Test
  public void testCoversEveryCANrangeSignal() {
    IOTestSupport.assertTelemetryCoverage(
        CoreCANrange.class,
        RangeSensorTelemetry.class,
        CANRANGE_SIGNAL_TO_FIELDS,
        IOTestSupport.COMMON_UNMAPPED_SIGNALS,
        IOTestSupport.COMMON_FIELDS_WITHOUT_SIGNAL);
  }
}
