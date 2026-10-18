package edu.wpi.team190.gompeilib.io.motor.config;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.wpilib.units.Units.Seconds;

import org.junit.jupiter.api.Test;

public class ClosedLoopRampsTest {

  @Test
  public void testNoneIsAllZero() {
    assertEquals(0.0, ClosedLoopRamps.NONE.dutyCycle().in(Seconds));
    assertEquals(0.0, ClosedLoopRamps.NONE.voltage().in(Seconds));
    assertEquals(0.0, ClosedLoopRamps.NONE.current().in(Seconds));
  }

  @Test
  public void testComponents() {
    ClosedLoopRamps ramps = new ClosedLoopRamps(Seconds.of(0.1), Seconds.of(0.25), Seconds.of(0.5));

    assertEquals(0.1, ramps.dutyCycle().in(Seconds));
    assertEquals(0.25, ramps.voltage().in(Seconds));
    assertEquals(0.5, ramps.current().in(Seconds));
  }
}
