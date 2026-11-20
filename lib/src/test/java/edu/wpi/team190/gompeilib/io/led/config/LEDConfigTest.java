package edu.wpi.team190.gompeilib.io.led.config;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Hertz;

import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import org.junit.jupiter.api.Test;

public class LEDConfigTest {

  @Test
  public void testDefaultsMatchTheDevice() {
    LEDConfig config = LEDConfig.builder().withCanId(2).withBus(CANBusId.named("rio")).build();

    assertEquals(2, config.canId);
    assertEquals(CANBusId.named("rio"), config.bus);
    assertEquals(LEDStripType.GRB, config.stripType);
    assertEquals(1.0, config.brightness);
    assertTrue(config.keepRunningOnLossOfSignal);
    assertTrue(config.enableFiveVoltRail);
    assertEquals(SupplyOutputMode.ON, config.supplyOutputMode);
    assertTrue(config.statusLedWhenActive);
    assertSame(LEDSignal.DEFAULT_RATES, config.signalRates);
  }

  @Test
  public void testEveryField() {
    SignalRates<LEDSignal, LEDSignalGroup> rates =
        LEDSignal.DEFAULT_RATES.withGroup(LEDSignalGroup.DIAGNOSTIC, Hertz.of(1.0));
    LEDConfig config =
        LEDConfig.builder()
            .withCanId(2)
            .withBus(CANBusId.named("rio"))
            .withStripType(LEDStripType.RGBW)
            .withBrightness(0.5)
            .withKeepRunningOnLossOfSignal(false)
            .withEnableFiveVoltRail(false)
            .withSupplyOutputMode(SupplyOutputMode.MODULATED)
            .withStatusLedWhenActive(false)
            .withSignalRates(rates)
            .build();

    assertEquals(LEDStripType.RGBW, config.stripType);
    assertEquals(0.5, config.brightness);
    assertFalse(config.keepRunningOnLossOfSignal);
    assertFalse(config.enableFiveVoltRail);
    assertEquals(SupplyOutputMode.MODULATED, config.supplyOutputMode);
    assertFalse(config.statusLedWhenActive);
    assertSame(rates, config.signalRates);
    assertEquals(3, config.toBuilder().withCanId(3).build().canId);
  }

  @Test
  public void shouldRequireIdAndBus() {
    assertThrows(
        NullPointerException.class,
        () -> LEDConfig.builder().withBus(CANBusId.named("rio")).build());
    assertThrows(NullPointerException.class, () -> LEDConfig.builder().withCanId(1).build());
  }
}
