package edu.wpi.team190.gompeilib.io.motor.config;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.wpilib.units.Units.Seconds;

import org.junit.jupiter.api.Test;

public class OpenLoopRampsTest {

  @Test
  public void testNoneIsAllZero() {
    assertEquals(0.0, OpenLoopRamps.NONE.dutyCycle().in(Seconds));
    assertEquals(0.0, OpenLoopRamps.NONE.voltage().in(Seconds));
    assertEquals(0.0, OpenLoopRamps.NONE.current().in(Seconds));
  }

  @Test
  public void testComponents() {
    OpenLoopRamps ramps = new OpenLoopRamps(Seconds.of(0.1), Seconds.of(0.25), Seconds.of(0.5));

    assertEquals(0.1, ramps.dutyCycle().in(Seconds));
    assertEquals(0.25, ramps.voltage().in(Seconds));
    assertEquals(0.5, ramps.current().in(Seconds));
  }
}
