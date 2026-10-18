package edu.wpi.team190.gompeilib.io.range.sim;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.range.config.RangeSensorConfig;
import edu.wpi.team190.gompeilib.io.range.config.RangeUpdateMode;
import edu.wpi.team190.gompeilib.io.range.inputs.MeasurementHealth;
import edu.wpi.team190.gompeilib.io.range.inputs.RangeSensorTelemetryAutoLogged;
import org.junit.jupiter.api.Test;

public class RangeSensorIOSimTest extends GompeiLibTestBase {

  private static RangeSensorConfig.RangeSensorConfigBuilder config() {
    return RangeSensorConfig.builder()
        .withCanId(1)
        .withBus(CANBusId.named("rio"))
        .withProximityThreshold(Meters.of(0.3))
        .withProximityHysteresis(Meters.of(0.05));
  }

  @Test
  public void testNothingInViewIsNotDetected() {
    RangeSensorIOSim sensor = new RangeSensorIOSim(config().build());
    RangeSensorTelemetryAutoLogged telemetry = new RangeSensorTelemetryAutoLogged();

    sensor.updateInputs(telemetry);

    assertTrue(sensor.isConnected());
    assertTrue(telemetry.connected);
    assertFalse(telemetry.detected);
    assertEquals(0.0, telemetry.signalStrength);
    assertEquals(MeasurementHealth.BAD, telemetry.measurementHealth);
    assertEquals(0.01, telemetry.measurementTime.in(Seconds), 1e-9);
    assertEquals(27.0, telemetry.fovRangeX.in(Degrees));
    assertTrue(telemetry.supplyVoltage.in(Volts) > 0);
  }

  @Test
  public void testDetectionFollowsThresholdAndHysteresis() {
    RangeSensorIOSim sensor = new RangeSensorIOSim(config().build());
    RangeSensorTelemetryAutoLogged telemetry = new RangeSensorTelemetryAutoLogged();

    sensor.setDistance(Meters.of(0.32));
    sensor.updateInputs(telemetry);
    assertFalse(telemetry.detected);
    assertEquals(0.32, telemetry.distance.in(Meters), 1e-9);
    assertEquals(RangeSensorIOSim.SIGNAL_STRENGTH, telemetry.signalStrength);
    assertEquals(MeasurementHealth.GOOD, telemetry.measurementHealth);

    sensor.setDistance(Meters.of(0.25));
    sensor.updateInputs(telemetry);
    assertTrue(telemetry.detected);

    // Still within the hysteresis band, so the target stays detected.
    sensor.setDistance(Meters.of(0.33));
    sensor.updateInputs(telemetry);
    assertTrue(telemetry.detected);

    sensor.setDistance(Meters.of(0.4));
    sensor.updateInputs(telemetry);
    assertFalse(telemetry.detected);

    sensor.setDistance(null);
    sensor.updateInputs(telemetry);
    assertFalse(telemetry.detected);
    assertEquals(0.0, telemetry.distance.in(Meters));
  }

  @Test
  public void testUserFrequencyModesSetTheMeasurementTime() {
    RangeSensorIOSim sensor =
        new RangeSensorIOSim(
            config()
                .withUpdateMode(RangeUpdateMode.LONG_RANGE_USER_FREQUENCY)
                .withUpdateFrequency(Hertz.of(20.0))
                .build());
    RangeSensorTelemetryAutoLogged telemetry = new RangeSensorTelemetryAutoLogged();

    sensor.updateInputs(telemetry);

    assertEquals(0.05, telemetry.measurementTime.in(Seconds), 1e-9);
  }
}
