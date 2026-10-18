package edu.wpi.team190.gompeilib.io.physics;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;

public class ElevatorMechanismSimTest extends GompeiLibTestBase {
  private static final double DRUM_RADIUS = 0.02;

  private static ElevatorMechanismSim elevator(boolean gravity) {
    return new ElevatorMechanismSim(
        DCMotor.getKrakenX60(2),
        10.0,
        Kilograms.of(5.0),
        Meters.of(DRUM_RADIUS),
        Meters.of(0.0),
        Meters.of(1.0),
        gravity,
        Meters.of(0.5));
  }

  @Test
  public void testReportsHeightAsDrumAngle() {
    ElevatorMechanismSim sim = elevator(false);

    assertEquals(0.5 / DRUM_RADIUS, sim.position().in(Radians), 1e-9);
    assertEquals(0.0, sim.velocity().in(RadiansPerSecond), 1e-9);
  }

  @Test
  public void testUpwardVoltageRaisesTheCarriage() {
    ElevatorMechanismSim sim = elevator(true);

    sim.setInputVoltage(Volts.of(12.0));
    for (int i = 0; i < 10; i++) {
      sim.update(Seconds.of(0.02));
    }

    assertTrue(sim.position().in(Radians) > 0.5 / DRUM_RADIUS);
    assertTrue(sim.velocity().in(RadiansPerSecond) > 0);
    assertTrue(sim.currentDraw().in(Amps) > 0);
  }
}
