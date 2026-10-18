package edu.wpi.team190.gompeilib.io.range.config;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import edu.wpi.team190.gompeilib.io.range.inputs.RangeSensorTelemetry;
import java.util.Map;
import org.junit.jupiter.api.Test;

public class RangeSensorConfigTest {

  @Test
  public void testDefaultsMatchTheDevice() {
    RangeSensorConfig config =
        RangeSensorConfig.builder().withCanId(6).withBus(CANBusId.named("rio")).build();

    assertEquals(6, config.canId);
    assertEquals(0.4, config.proximityThreshold.in(Meters));
    assertEquals(0.01, config.proximityHysteresis.in(Meters));
    assertEquals(2500.0, config.minSignalStrength);
    assertEquals(RangeUpdateMode.SHORT_RANGE_100HZ, config.updateMode);
    assertEquals(50.0, config.updateFrequency.in(Hertz));
    assertEquals(0.0, config.fovCenterX.in(Degrees));
    assertEquals(0.0, config.fovCenterY.in(Degrees));
    assertEquals(27.0, config.fovRangeX.in(Degrees));
    assertEquals(27.0, config.fovRangeY.in(Degrees));
    assertSame(SignalRates.DEFAULT, config.signalRates);
  }

  @Test
  public void testEveryField() {
    SignalRates rates = new SignalRates(Hertz.of(100.0), Hertz.of(4.0), Map.of());
    RangeSensorConfig config =
        RangeSensorConfig.builder()
            .withCanId(6)
            .withBus(CANBusId.named("rio"))
            .withProximityThreshold(Meters.of(0.2))
            .withProximityHysteresis(Meters.of(0.05))
            .withMinSignalStrength(1000.0)
            .withUpdateMode(RangeUpdateMode.LONG_RANGE_USER_FREQUENCY)
            .withUpdateFrequency(Hertz.of(25.0))
            .withFovCenterX(Degrees.of(1.0))
            .withFovCenterY(Degrees.of(-1.0))
            .withFovRangeX(Degrees.of(10.0))
            .withFovRangeY(Degrees.of(12.0))
            .withSignalRates(rates)
            .build();

    assertEquals(0.2, config.proximityThreshold.in(Meters));
    assertEquals(0.05, config.proximityHysteresis.in(Meters));
    assertEquals(1000.0, config.minSignalStrength);
    assertEquals(RangeUpdateMode.LONG_RANGE_USER_FREQUENCY, config.updateMode);
    assertEquals(25.0, config.updateFrequency.in(Hertz));
    assertEquals(1.0, config.fovCenterX.in(Degrees));
    assertEquals(-1.0, config.fovCenterY.in(Degrees));
    assertEquals(10.0, config.fovRangeX.in(Degrees));
    assertEquals(12.0, config.fovRangeY.in(Degrees));
    assertSame(rates, config.signalRates);
    assertEquals(7, config.toBuilder().withCanId(7).build().canId);
  }

  @Test
  public void testControlFieldsAreRealTelemetryFields() {
    assertTrue(
        IOTestSupport.publicFieldNames(RangeSensorTelemetry.class)
            .containsAll(RangeSensorConfig.CONTROL_FIELDS));
  }

  @Test
  public void shouldRequireIdAndBus() {
    assertThrows(
        NullPointerException.class,
        () -> RangeSensorConfig.builder().withBus(CANBusId.named("rio")).build());
    assertThrows(
        NullPointerException.class, () -> RangeSensorConfig.builder().withCanId(1).build());
  }
}
