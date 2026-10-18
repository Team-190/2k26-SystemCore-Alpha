package edu.wpi.team190.gompeilib.io.motor.config;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Gains;
import org.junit.jupiter.api.Test;

public class SlotConfigTest extends GompeiLibTestBase {
  private Gains gains() {
    return Gains.builder().withKP(2.0).withKG(0.3).build();
  }

  @Test
  public void testComponents() {
    Gains gains = gains();
    SlotConfig slot = new SlotConfig(gains, GravityType.COSINE);

    assertSame(gains, slot.gains());
    assertEquals(GravityType.COSINE, slot.gravityType());
  }

  @Test
  public void testDefaultsToConstantGravity() {
    assertEquals(GravityType.CONSTANT, new SlotConfig(gains()).gravityType());
  }

  @Test
  public void shouldRejectNulls() {
    assertThrows(NullPointerException.class, () -> new SlotConfig(null, GravityType.CONSTANT));
    assertThrows(NullPointerException.class, () -> new SlotConfig(gains(), null));
  }
}
