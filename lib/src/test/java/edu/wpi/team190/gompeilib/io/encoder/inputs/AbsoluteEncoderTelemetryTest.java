package edu.wpi.team190.gompeilib.io.encoder.inputs;

import static java.util.Map.entry;

import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import edu.wpi.team190.gompeilib.io.IOTestSupport;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

public class AbsoluteEncoderTelemetryTest {

  /** Which {@link AbsoluteEncoderTelemetry} fields each CANcoder status signal fills. */
  public static final Map<String, List<String>> CANCODER_SIGNAL_TO_FIELDS =
      Map.ofEntries(
          entry("Version", List.of("firmwareVersion")),
          entry("IsProLicensed", List.of("isProLicensed")),
          entry("AbsolutePosition", List.of("absolutePosition")),
          entry("Position", List.of("position")),
          entry("PositionSinceBoot", List.of("positionSinceBoot")),
          entry("Velocity", List.of("velocity")),
          entry("UnfilteredVelocity", List.of("unfilteredVelocity")),
          entry("MagnetHealth", List.of("magnetHealth")),
          entry("SupplyVoltage", List.of("supplyVoltage")));

  @Test
  public void testDefaultValues() throws IllegalAccessException {
    IOTestSupport.assertZeroDefaults(new AbsoluteEncoderTelemetry());
  }

  @Test
  public void testCoversEveryCANcoderSignal() {
    IOTestSupport.assertTelemetryCoverage(
        CoreCANcoder.class,
        AbsoluteEncoderTelemetry.class,
        CANCODER_SIGNAL_TO_FIELDS,
        IOTestSupport.COMMON_UNMAPPED_SIGNALS,
        IOTestSupport.COMMON_FIELDS_WITHOUT_SIGNAL);
  }
}
