package edu.wpi.team190.gompeilib.io.physics;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;

public class ArmMechanismSimTest extends GompeiLibTestBase {

  private static ArmMechanismSim arm(boolean gravity) {
    return new ArmMechanismSim(
        DCMotor.getKrakenX60(1),
        50.0,
        KilogramSquareMeters.of(0.1),
        Meters.of(0.5),
        Degrees.of(-90.0),
        Degrees.of(90.0),
        gravity,
        Degrees.of(0.0));
  }

  @Test
  public void testGravityPullsTheArmDown() {
    ArmMechanismSim sim = arm(true);

    for (int i = 0; i < 10; i++) {
      sim.update(Seconds.of(0.02));
    }

    assertTrue(sim.position().in(Radians) < 0);
    assertTrue(sim.velocity().in(RadiansPerSecond) < 0);
  }

  @Test
  public void testVoltageRaisesTheArm() {
    ArmMechanismSim sim = arm(false);

    sim.setInputVoltage(Volts.of(6.0));
    for (int i = 0; i < 10; i++) {
      sim.update(Seconds.of(0.02));
    }

    assertTrue(sim.position().in(Radians) > 0);
    assertTrue(sim.currentDraw().in(Amps) > 0);
  }
}
