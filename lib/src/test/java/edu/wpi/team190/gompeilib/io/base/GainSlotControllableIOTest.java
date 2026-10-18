package edu.wpi.team190.gompeilib.io.base;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.TestGains;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import org.junit.jupiter.api.Test;

public class GainSlotControllableIOTest extends GompeiLibTestBase {

  @Test
  public void testDefaultMethods() {
    GainSlotControllableIO io = new GainSlotControllableIO() {};

    assertDoesNotThrow(
        () -> io.updateGains(SlotConfig.position(TestGains.positionKP(1.0)), GainSlot.ONE));
  }
}
