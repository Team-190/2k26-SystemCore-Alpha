package edu.wpi.team190.gompeilib.io.motor.config;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.TestGains;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import org.junit.jupiter.api.Test;

public class SlotConfigTest extends GompeiLibTestBase {
  @Test
  public void testComponents() {
    var gains = TestGains.position(2.0, 0, 0, 0, 0, 0, 0.3);
    SlotConfig slot = SlotConfig.position(gains, GravityType.COSINE);

    assertSame(gains, slot.gains());
    assertEquals(GravityType.COSINE, slot.gravityType());
    assertInstanceOf(SlotConfig.Position.class, slot);
  }

  @Test
  public void testDefaultsToConstantGravity() {
    assertEquals(
        GravityType.CONSTANT, SlotConfig.position(TestGains.positionKP(1.0)).gravityType());
    assertEquals(
        GravityType.CONSTANT,
        SlotConfig.velocity(TestGains.velocity(1.0, 0, 0, 0, 0, 0, 0)).gravityType());
  }

  @Test
  public void shouldReportTheGainsOutputType() {
    assertEquals(OutputType.VOLTAGE, SlotConfig.position(TestGains.positionKP(1.0)).outputType());
    assertEquals(
        OutputType.CURRENT, SlotConfig.velocity(Gains.angularVelocity(Amps).build()).outputType());
    assertEquals(
        OutputType.DUTY_CYCLE,
        SlotConfig.position(Gains.angularPosition(Percent).build()).outputType());
  }

  @Test
  public void shouldRejectGainsAMotorCantOutput() {
    var chassis = Gains.angularPosition(RadiansPerSecond).build();
    assertThrows(IllegalArgumentException.class, () -> SlotConfig.position(chassis));
    var chassisVelocity = Gains.angularVelocity(RadiansPerSecond).build();
    assertThrows(IllegalArgumentException.class, () -> SlotConfig.velocity(chassisVelocity));
  }

  @Test
  public void shouldCheckTheRequestsOutputType() {
    SlotConfig current = SlotConfig.velocity(Gains.angularVelocity(Amps).build());
    SlotConfig.requireOutputType(current, OutputType.CURRENT, "Drive");
    IllegalArgumentException error =
        assertThrows(
            IllegalArgumentException.class,
            () -> SlotConfig.requireOutputType(current, OutputType.VOLTAGE, "Drive"));
    assertTrue(error.getMessage().startsWith("Drive"));
  }

  @Test
  public void shouldRejectNulls() {
    assertThrows(
        NullPointerException.class, () -> new SlotConfig.Position(null, GravityType.CONSTANT));
    assertThrows(
        NullPointerException.class, () -> new SlotConfig.Position(TestGains.positionKP(1.0), null));
    assertThrows(
        NullPointerException.class, () -> new SlotConfig.Velocity(null, GravityType.CONSTANT));
    assertThrows(
        NullPointerException.class,
        () -> new SlotConfig.Velocity(TestGains.velocity(1.0, 0, 0, 0, 0, 0, 0), null));
  }
}
