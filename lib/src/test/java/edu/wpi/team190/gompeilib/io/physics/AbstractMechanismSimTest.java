package edu.wpi.team190.gompeilib.io.physics;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;

public class AbstractMechanismSimTest extends GompeiLibTestBase {

  /** Velocity grows by the applied voltage per second, so acceleration equals the voltage. */
  private static final class LinearSim extends AbstractMechanismSim {
    private double volts;
    private double velocity;

    LinearSim() {
      super(DCMotor.getKrakenX60(1));
    }

    @Override
    protected void applyVoltage(double volts) {
      this.volts = volts;
    }

    @Override
    protected void step(double seconds) {
      velocity += volts * seconds;
    }

    @Override
    public Angle position() {
      return Radians.of(0.0);
    }

    @Override
    public AngularVelocity velocity() {
      return RadiansPerSecond.of(velocity);
    }

    @Override
    public Current currentDraw() {
      return Amps.of(0.0);
    }
  }

  @Test
  public void testFirstRegisteredMotorDrives() {
    LinearSim sim = new LinearSim();
    Object first = new Object();
    Object second = new Object();

    assertNull(sim.driver());
    assertEquals(1, sim.motorCount());
    assertTrue(sim.claimDriver(first));
    assertFalse(sim.claimDriver(second));
    assertTrue(sim.claimDriver(first));
    assertSame(first, sim.driver());
    assertEquals(2, sim.motorCount());
  }

  @Test
  public void testInputVoltageAndAcceleration() {
    LinearSim sim = new LinearSim();

    assertEquals(0.0, sim.inputVoltage().in(Volts));
    assertEquals(0.0, sim.acceleration().in(RadiansPerSecondPerSecond));

    sim.setInputVoltage(Volts.of(3.0));
    sim.update(Seconds.of(0.02));

    assertEquals(3.0, sim.inputVoltage().in(Volts));
    assertEquals(3.0, sim.acceleration().in(RadiansPerSecondPerSecond), 1e-9);
    assertNotNull(sim.gearbox());
  }
}
