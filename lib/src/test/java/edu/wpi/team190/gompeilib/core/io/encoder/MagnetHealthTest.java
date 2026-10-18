package edu.wpi.team190.gompeilib.core.io.encoder;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class MagnetHealthTest {

  @Test
  public void testEnumConstants() {
    for (MagnetHealth health : MagnetHealth.values()) {
      assertEquals(health, MagnetHealth.valueOf(health.name()));
    }
    assertEquals(MagnetHealth.UNKNOWN, MagnetHealth.valueOf("UNKNOWN"));
  }
}
