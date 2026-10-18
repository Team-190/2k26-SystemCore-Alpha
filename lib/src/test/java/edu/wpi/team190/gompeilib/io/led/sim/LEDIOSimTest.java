package edu.wpi.team190.gompeilib.io.led.sim;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.led.inputs.LEDTelemetryAutoLogged;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDAnimation;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDColor;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDRange;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDRequest;
import java.util.Optional;
import org.junit.jupiter.api.Test;

public class LEDIOSimTest extends GompeiLibTestBase {
  private static final LEDColor RED = new LEDColor(255, 0, 0);
  private static final LEDColor OFF = new LEDColor(0, 0, 0, 0);

  private static LEDAnimation.Rainbow rainbow(int slot) {
    return new LEDAnimation.Rainbow(
        new LEDRange(0, 3),
        slot,
        1.0,
        edu.wpi.team190.gompeilib.io.led.outputs.AnimationDirection.FORWARD,
        Hertz.of(60.0),
        Hertz.of(20.0));
  }

  @Test
  public void testSolidSetsTheRangeClampedToTheStrip() {
    LEDIOSim leds = new LEDIOSim(10);

    leds.setControl(new LEDRequest.Solid(new LEDRange(8, 20), RED));

    assertEquals(OFF, leds.color(7));
    assertEquals(RED, leds.color(8));
    assertEquals(RED, leds.color(9));
    assertTrue(leds.isConnected());
  }

  @Test
  public void testAnimationsOccupyAndClearSlots() {
    LEDIOSim leds = new LEDIOSim(4);

    leds.setControl(rainbow(3));
    assertEquals(Optional.of(rainbow(3)), leds.animation(3));
    assertEquals(Optional.empty(), leds.animation(0));

    leds.setControl(new LEDAnimation.ClearAnimation(3, Hertz.of(20.0)));
    assertEquals(Optional.empty(), leds.animation(3));
  }

  @Test
  public void shouldRejectOutOfRangeSlots() {
    LEDIOSim leds = new LEDIOSim(4);

    assertThrows(IllegalArgumentException.class, () -> leds.setControl(rainbow(8)));
    assertThrows(IllegalArgumentException.class, () -> leds.setControl(rainbow(-1)));
  }

  @Test
  public void testTelemetry() {
    LEDIOSim leds = new LEDIOSim(4);
    LEDTelemetryAutoLogged telemetry = new LEDTelemetryAutoLogged();

    leds.setControl(new LEDRequest.SupplyModulation(0.75, Hertz.of(100.0)));
    leds.updateInputs(telemetry);

    assertTrue(telemetry.connected);
    assertEquals(0.75, telemetry.supplyModulation);
    assertEquals(5.0, telemetry.fiveVoltRailVoltage.in(Volts));
    assertTrue(telemetry.supplyVoltage.in(Volts) > 0);
    assertEquals(LEDIOSim.ANIMATION_SLOTS, telemetry.maxSimultaneousAnimations);
  }
}
