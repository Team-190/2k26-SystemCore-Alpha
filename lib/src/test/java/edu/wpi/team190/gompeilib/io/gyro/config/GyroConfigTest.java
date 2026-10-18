package edu.wpi.team190.gompeilib.io.gyro.config;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import edu.wpi.team190.gompeilib.io.gyro.inputs.GyroTelemetry;
import java.util.Map;
import org.junit.jupiter.api.Test;

public class GyroConfigTest {

  @Test
  public void testDefaultsMatchTheDevice() {
    GyroConfig config = GyroConfig.builder().withCanId(1).withBus(CANBusId.named("Drive")).build();

    assertEquals(1, config.canId);
    assertEquals(CANBusId.named("Drive"), config.bus);
    assertEquals(0.0, config.mountPoseYaw.in(Degrees));
    assertEquals(0.0, config.mountPosePitch.in(Degrees));
    assertEquals(0.0, config.mountPoseRoll.in(Degrees));
    assertFalse(config.enableCompass);
    assertFalse(config.disableTemperatureCompensation);
    assertFalse(config.disableNoMotionCalibration);
    assertSame(SignalRates.DEFAULT, config.signalRates);
  }

  @Test
  public void testEveryField() {
    SignalRates rates =
        new SignalRates(Hertz.of(50.0), Hertz.of(4.0), Map.of("yawPosition", Hertz.of(250.0)));
    GyroConfig config =
        GyroConfig.builder()
            .withCanId(1)
            .withBus(CANBusId.named("Drive"))
            .withMountPoseYaw(Degrees.of(90.0))
            .withMountPosePitch(Degrees.of(1.0))
            .withMountPoseRoll(Degrees.of(-2.0))
            .withEnableCompass(true)
            .withDisableTemperatureCompensation(true)
            .withDisableNoMotionCalibration(true)
            .withSignalRates(rates)
            .build();

    assertEquals(90.0, config.mountPoseYaw.in(Degrees));
    assertEquals(1.0, config.mountPosePitch.in(Degrees));
    assertEquals(-2.0, config.mountPoseRoll.in(Degrees));
    assertTrue(config.enableCompass);
    assertTrue(config.disableTemperatureCompensation);
    assertTrue(config.disableNoMotionCalibration);
    assertSame(rates, config.signalRates);
    assertEquals(2, config.toBuilder().withCanId(2).build().canId);
  }

  @Test
  public void testControlFieldsAreRealTelemetryFields() {
    assertTrue(
        IOTestSupport.publicFieldNames(GyroTelemetry.class).containsAll(GyroConfig.CONTROL_FIELDS));
  }

  @Test
  public void shouldRequireIdAndBus() {
    assertThrows(
        NullPointerException.class,
        () -> GyroConfig.builder().withBus(CANBusId.named("Drive")).build());
    assertThrows(NullPointerException.class, () -> GyroConfig.builder().withCanId(1).build());
  }
}
