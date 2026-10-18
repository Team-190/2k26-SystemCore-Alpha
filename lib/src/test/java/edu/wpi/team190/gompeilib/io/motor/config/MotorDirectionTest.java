package edu.wpi.team190.gompeilib.io.motor.config;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class MotorDirectionTest {

  @Test
  public void testConstants() {
    IOTestSupport.assertEnumRoundTrip(
        MotorDirection.class, "COUNTER_CLOCKWISE_POSITIVE", "CLOCKWISE_POSITIVE");
  }

  @Test
  public void testOpposite() {
    assertEquals(
        MotorDirection.CLOCKWISE_POSITIVE, MotorDirection.COUNTER_CLOCKWISE_POSITIVE.opposite());
    assertEquals(
        MotorDirection.COUNTER_CLOCKWISE_POSITIVE, MotorDirection.CLOCKWISE_POSITIVE.opposite());
  }
}
