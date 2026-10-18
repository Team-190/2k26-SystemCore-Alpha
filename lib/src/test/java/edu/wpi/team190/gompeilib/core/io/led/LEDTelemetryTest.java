package edu.wpi.team190.gompeilib.core.io.led;

import static java.util.Map.entry;

import com.ctre.phoenix6.hardware.core.CoreCANdle;
import edu.wpi.team190.gompeilib.core.io.IOTestSupport;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

public class LEDTelemetryTest {

  /** Which {@link LEDTelemetry} fields each CANdle status signal fills. */
  public static final Map<String, List<String>> CANDLE_SIGNAL_TO_FIELDS =
      Map.ofEntries(
          entry("Version", List.of("firmwareVersion")),
          entry("IsProLicensed", List.of("isProLicensed")),
          entry("SupplyVoltage", List.of("supplyVoltage")),
          entry("FiveVRailVoltage", List.of("fiveVoltRailVoltage")),
          entry("OutputCurrent", List.of("outputCurrent")),
          entry("DeviceTemp", List.of("deviceTemperature")),
          entry("VBatModulation", List.of("supplyModulation")),
          entry("MaxSimultaneousAnimationCount", List.of("maxSimultaneousAnimations")));

  @Test
  public void testDefaultValues() throws IllegalAccessException {
    IOTestSupport.assertZeroDefaults(new LEDTelemetry());
  }

  @Test
  public void testCoversEveryCANdleSignal() {
    IOTestSupport.assertTelemetryCoverage(
        CoreCANdle.class,
        LEDTelemetry.class,
        CANDLE_SIGNAL_TO_FIELDS,
        IOTestSupport.COMMON_UNMAPPED_SIGNALS,
        IOTestSupport.COMMON_FIELDS_WITHOUT_SIGNAL);
  }
}
