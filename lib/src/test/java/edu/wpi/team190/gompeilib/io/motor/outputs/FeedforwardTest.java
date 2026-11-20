package edu.wpi.team190.gompeilib.io.motor.outputs;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import org.wpilib.units.Units;

public class FeedforwardTest {

  @Test
  public void shouldCarryItsOutputType() {
    assertEquals(OutputType.VOLTAGE, Feedforward.volts(Units.Volts.of(1.0)).type());
    assertEquals(OutputType.CURRENT, Feedforward.amps(Units.Amps.of(1.0)).type());
    assertEquals(OutputType.DUTY_CYCLE, Feedforward.dutyCycle(0.1).type());
  }

  @Test
  public void shouldHoldItsValue() {
    assertEquals(
        Units.Volts.of(2.0),
        ((Feedforward.Volts) Feedforward.volts(Units.Volts.of(2.0))).voltage());
    assertEquals(
        Units.Amps.of(3.0), ((Feedforward.Amps) Feedforward.amps(Units.Amps.of(3.0))).current());
    assertEquals(0.25, ((Feedforward.DutyCycle) Feedforward.dutyCycle(0.25)).dutyCycle());
  }

  @Test
  public void shouldBuildZeroOfEachType() {
    assertEquals(new Feedforward.Volts(Units.Volts.of(0.0)), Feedforward.zero(OutputType.VOLTAGE));
    assertEquals(new Feedforward.Amps(Units.Amps.of(0.0)), Feedforward.zero(OutputType.CURRENT));
    assertEquals(new Feedforward.DutyCycle(0.0), Feedforward.zero(OutputType.DUTY_CYCLE));
  }

  @Test
  public void shouldRejectNulls() {
    assertThrows(NullPointerException.class, () -> Feedforward.volts(null));
    assertThrows(NullPointerException.class, () -> Feedforward.amps(null));
    assertThrows(NullPointerException.class, () -> Feedforward.zero(null));
  }
}
