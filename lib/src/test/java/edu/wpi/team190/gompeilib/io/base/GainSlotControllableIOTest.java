package edu.wpi.team190.gompeilib.io.base;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import org.junit.jupiter.api.Test;

public class GainSlotControllableIOTest extends GompeiLibTestBase {

  @Test
  public void testDefaultMethods() {
    GainSlotControllableIO io = new GainSlotControllableIO() {};

    assertDoesNotThrow(
        () ->
            io.updateGains(
                Gains.builder().withKP(1.0).withKI(0.0).withKD(0.0).build(), GainSlot.ONE));
  }
}
