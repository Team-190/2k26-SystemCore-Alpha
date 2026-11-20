package edu.wpi.team190.gompeilib.io.encoder.config;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import org.junit.jupiter.api.Test;

public class AbsoluteEncoderConfigTest {

  @Test
  public void testDefaultsMatchTheDevice() {
    AbsoluteEncoderConfig config =
        AbsoluteEncoderConfig.builder().withCanId(5).withBus(CANBusId.named("rio")).build();

    assertEquals(5, config.canId);
    assertEquals(CANBusId.named("rio"), config.bus);
    assertEquals(SensorDirection.COUNTER_CLOCKWISE_POSITIVE, config.direction);
    assertEquals(0.0, config.magnetOffset.in(Rotations));
    assertEquals(0.5, config.discontinuityPoint.in(Rotations));
    assertSame(AbsoluteEncoderSignal.DEFAULT_RATES, config.signalRates);
  }

  @Test
  public void testEveryField() {
    SignalRates<AbsoluteEncoderSignal, AbsoluteEncoderSignalGroup> rates =
        AbsoluteEncoderSignal.DEFAULT_RATES.withGroup(
            AbsoluteEncoderSignalGroup.DIAGNOSTIC, Hertz.of(10.0));
    AbsoluteEncoderConfig config =
        AbsoluteEncoderConfig.builder()
            .withCanId(5)
            .withBus(CANBusId.named("rio"))
            .withDirection(SensorDirection.CLOCKWISE_POSITIVE)
            .withMagnetOffset(Rotations.of(-0.25))
            .withDiscontinuityPoint(Rotations.of(1.0))
            .withSignalRates(rates)
            .build();

    assertEquals(SensorDirection.CLOCKWISE_POSITIVE, config.direction);
    assertEquals(-0.25, config.magnetOffset.in(Rotations));
    assertEquals(1.0, config.discontinuityPoint.in(Rotations));
    assertSame(rates, config.signalRates);
    assertEquals(7, config.toBuilder().withCanId(7).build().canId);
  }

  @Test
  public void shouldRequireIdAndBus() {
    assertThrows(
        NullPointerException.class,
        () -> AbsoluteEncoderConfig.builder().withBus(CANBusId.named("rio")).build());
    assertThrows(
        NullPointerException.class, () -> AbsoluteEncoderConfig.builder().withCanId(1).build());
  }
}
