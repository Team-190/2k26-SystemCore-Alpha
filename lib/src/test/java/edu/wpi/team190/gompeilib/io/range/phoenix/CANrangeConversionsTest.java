package edu.wpi.team190.gompeilib.io.range.phoenix;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.signals.MeasurementHealthValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.range.config.RangeSensorConfig;
import edu.wpi.team190.gompeilib.io.range.config.RangeUpdateMode;
import edu.wpi.team190.gompeilib.io.range.inputs.MeasurementHealth;
import org.junit.jupiter.api.Test;

public class CANrangeConversionsTest {

  private static RangeSensorConfig.RangeSensorConfigBuilder config() {
    return RangeSensorConfig.builder().withCanId(1).withBus(CANBusId.named("rio"));
  }

  @Test
  public void testToConfiguration() {
    CANrangeConfiguration canrange =
        CANrangeConversions.toConfiguration(
            config()
                .withProximityThreshold(Meters.of(0.2))
                .withProximityHysteresis(Meters.of(0.05))
                .withMinSignalStrength(1000.0)
                .withUpdateMode(RangeUpdateMode.LONG_RANGE_USER_FREQUENCY)
                .withUpdateFrequency(Hertz.of(25.0))
                .withFovCenterX(Degrees.of(1.0))
                .withFovCenterY(Degrees.of(-1.0))
                .withFovRangeX(Degrees.of(10.0))
                .withFovRangeY(Degrees.of(12.0))
                .build());

    assertEquals(0.2, canrange.ProximityParams.ProximityThreshold, 1e-9);
    assertEquals(0.05, canrange.ProximityParams.ProximityHysteresis, 1e-9);
    assertEquals(1000.0, canrange.ProximityParams.MinSignalStrengthForValidMeasurement, 1e-9);
    assertEquals(UpdateModeValue.LongRangeUserFreq, canrange.ToFParams.UpdateMode);
    assertEquals(25.0, canrange.ToFParams.UpdateFrequency, 1e-9);
    assertEquals(1.0, canrange.FovParams.FOVCenterX, 1e-9);
    assertEquals(-1.0, canrange.FovParams.FOVCenterY, 1e-9);
    assertEquals(10.0, canrange.FovParams.FOVRangeX, 1e-9);
    assertEquals(12.0, canrange.FovParams.FOVRangeY, 1e-9);
    assertEquals(
        UpdateModeValue.ShortRangeUserFreq,
        CANrangeConversions.toConfiguration(
                config().withUpdateMode(RangeUpdateMode.SHORT_RANGE_USER_FREQUENCY).build())
            .ToFParams
            .UpdateMode);
  }

  @Test
  public void testDefaultConfigurationMatchesTheDevice() {
    CANrangeConfiguration converted = CANrangeConversions.toConfiguration(config().build());
    CANrangeConfiguration factory = new CANrangeConfiguration();

    assertEquals(
        factory.ProximityParams.ProximityThreshold,
        converted.ProximityParams.ProximityThreshold,
        1e-9);
    assertEquals(
        factory.ProximityParams.ProximityHysteresis,
        converted.ProximityParams.ProximityHysteresis,
        1e-9);
    assertEquals(
        factory.ProximityParams.MinSignalStrengthForValidMeasurement,
        converted.ProximityParams.MinSignalStrengthForValidMeasurement,
        1e-9);
    assertEquals(factory.ToFParams.UpdateMode, converted.ToFParams.UpdateMode);
    assertEquals(factory.ToFParams.UpdateFrequency, converted.ToFParams.UpdateFrequency, 1e-9);
    assertEquals(factory.FovParams.FOVRangeX, converted.FovParams.FOVRangeX, 1e-9);
    assertEquals(factory.FovParams.FOVRangeY, converted.FovParams.FOVRangeY, 1e-9);
  }

  @Test
  public void testToMeasurementHealth() {
    assertEquals(
        MeasurementHealth.GOOD,
        CANrangeConversions.toMeasurementHealth(MeasurementHealthValue.Good));
    assertEquals(
        MeasurementHealth.LIMITED,
        CANrangeConversions.toMeasurementHealth(MeasurementHealthValue.Limited));
    assertEquals(
        MeasurementHealth.BAD, CANrangeConversions.toMeasurementHealth(MeasurementHealthValue.Bad));
  }
}
