package edu.wpi.team190.gompeilib.core.io.base;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import org.junit.jupiter.api.Test;

public class GainSlotControllableIOTest {

  @Test
  public void testDefaultMethods() {
    GainSlotControllableIO io = new GainSlotControllableIO() {};

    assertDoesNotThrow(
        () ->
            io.updateGains(
                Gains.builder().withKP(1.0).withKI(0.0).withKD(0.0).build(), GainSlot.ONE));
  }
}
