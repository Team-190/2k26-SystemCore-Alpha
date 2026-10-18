package edu.wpi.team190.gompeilib.io.physics;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;

public class DCMotorMechanismSimTest extends GompeiLibTestBase {

  @Test
  public void testPositiveVoltageSpinsForward() {
    DCMotorMechanismSim sim =
        new DCMotorMechanismSim(DCMotor.getKrakenX60(1), KilogramSquareMeters.of(0.001), 5.0);

    sim.setInputVoltage(Volts.of(12.0));
    for (int i = 0; i < 50; i++) {
      sim.update(Seconds.of(0.02));
    }

    assertTrue(sim.velocity().in(RadiansPerSecond) > 0);
    assertTrue(sim.position().in(Radians) > 0);
    assertTrue(sim.currentDraw().in(Amps) > 0);
  }

  @Test
  public void testFreeSpeedMatchesTheGearing() {
    DCMotor motor = DCMotor.getKrakenX60(1);
    DCMotorMechanismSim sim = new DCMotorMechanismSim(motor, KilogramSquareMeters.of(0.0001), 4.0);

    sim.setInputVoltage(Volts.of(12.0));
    for (int i = 0; i < 500; i++) {
      sim.update(Seconds.of(0.02));
    }

    assertEquals(
        motor.freeSpeed / 4.0, sim.velocity().in(RadiansPerSecond), motor.freeSpeed * 0.01);
  }
}
