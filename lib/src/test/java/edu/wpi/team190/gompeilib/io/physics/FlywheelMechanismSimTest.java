package edu.wpi.team190.gompeilib.io.physics;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;

public class FlywheelMechanismSimTest extends GompeiLibTestBase {

  @Test
  public void testIntegratesPositionFromVelocity() {
    FlywheelMechanismSim sim =
        new FlywheelMechanismSim(DCMotor.getKrakenX60(2), KilogramSquareMeters.of(0.002), 1.0);

    assertEquals(0.0, sim.position().in(Radians));
    sim.setInputVoltage(Volts.of(6.0));
    double expected = 0.0;
    for (int i = 0; i < 50; i++) {
      sim.update(Seconds.of(0.02));
      expected += sim.velocity().in(RadiansPerSecond) * 0.02;
    }

    assertTrue(sim.velocity().in(RadiansPerSecond) > 0);
    assertEquals(expected, sim.position().in(Radians), 1e-9);
    assertTrue(sim.currentDraw().in(Amps) > 0);
  }
}
