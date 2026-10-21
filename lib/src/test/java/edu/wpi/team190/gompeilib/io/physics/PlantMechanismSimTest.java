package edu.wpi.team190.gompeilib.io.physics;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.motor.config.GravityType;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.VoltageUnit;

public class PlantMechanismSimTest extends GompeiLibTestBase {
  private static final double EPSILON = 1e-9;
  private static final double DT = 0.001;
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60(1);
  private static final double GEAR_RATIO = 10.0;

  private static PlantCharacterization.Builder<VoltageUnit> volts(double kS, double kV, double kA) {
    return PlantCharacterization.voltage()
        .withKS(Volts.of(kS))
        .withKV(Volts.per(RadiansPerSecond).ofNative(kV))
        .withKA(Volts.per(RadiansPerSecondPerSecond).ofNative(kA));
  }

  private static PlantMechanismSim sim(PlantCharacterization<?> plant) {
    return new PlantMechanismSim(GEARBOX, GEAR_RATIO, plant);
  }

  private static void run(PlantMechanismSim sim, double volts, double seconds) {
    sim.setInputVoltage(Volts.of(volts));
    long steps = Math.round(seconds / DT);
    for (long i = 0; i < steps; i++) {
      sim.update(Seconds.of(DT));
    }
  }

  @Test
  public void testReachesSteadyStateWithItsTimeConstant() {
    // kS = 1 V, kV = 2 V/(rad/s), kA = 0.2 V/(rad/s²): 7 V settles at 3 rad/s, τ = 0.1 s
    PlantMechanismSim sim = sim(volts(1.0, 2.0, 0.2).build());

    run(sim, 7.0, 0.1);
    assertEquals(3.0 * (1 - Math.exp(-1)), sim.velocity().in(RadiansPerSecond), 1e-6);

    run(sim, 7.0, 2.0);
    assertEquals(3.0, sim.velocity().in(RadiansPerSecond), 1e-6);
    assertTrue(sim.position().in(Radians) > 0);
    assertEquals(7.0, sim.inputVoltage().in(Volts));
  }

  @Test
  public void testStaticFrictionHoldsItStill() {
    PlantMechanismSim sim = sim(volts(1.0, 2.0, 0.2).build());

    run(sim, 0.9, 1.0);
    assertEquals(0.0, sim.velocity().in(RadiansPerSecond));
    assertEquals(0.0, sim.position().in(Radians));

    run(sim, -0.9, 1.0);
    assertEquals(0.0, sim.velocity().in(RadiansPerSecond));
  }

  @Test
  public void testFrictionStopsACoastingMechanismWithoutReversingIt() {
    PlantMechanismSim sim = sim(volts(1.0, 2.0, 0.2).build());

    run(sim, -7.0, 1.0);
    assertTrue(sim.velocity().in(RadiansPerSecond) < -2.9);
    run(sim, 0.0, 1.0);
    assertEquals(0.0, sim.velocity().in(RadiansPerSecond));
    double stopped = sim.position().in(Radians);
    run(sim, 0.0, 0.5);
    assertEquals(stopped, sim.position().in(Radians));
  }

  @Test
  public void testWithoutDampingItAcceleratesUniformly() {
    PlantMechanismSim sim = sim(volts(1.0, 0.0, 0.5).build());

    run(sim, 3.0, 0.5);
    assertEquals(2.0, sim.velocity().in(RadiansPerSecond), 1e-6);
    assertEquals(0.5 * 4.0 * 0.25, sim.position().in(Radians), 1e-6);
  }

  @Test
  public void testGravityPullsAnArmThatTheVoltageDoesntHold() {
    PlantCharacterization<VoltageUnit> plant =
        volts(0.0, 2.0, 0.2).withKG(Volts.of(1.0)).withGravityType(GravityType.COSINE).build();
    PlantMechanismSim held =
        new PlantMechanismSim(
            GEARBOX, GEAR_RATIO, plant, Degrees.of(-90), Degrees.of(90), Degrees.of(60));

    run(held, 0.5, 1.0);
    assertEquals(Math.PI / 3, held.position().in(Radians), EPSILON);
    assertEquals(0.0, held.velocity().in(RadiansPerSecond), EPSILON);

    PlantMechanismSim falling =
        new PlantMechanismSim(
            GEARBOX, GEAR_RATIO, plant, Degrees.of(-90), Degrees.of(90), Degrees.of(60));
    run(falling, 0.0, 0.1);
    assertTrue(falling.position().in(Radians) < Math.PI / 3);
  }

  @Test
  public void testHardStopsClampThePosition() {
    PlantMechanismSim sim =
        new PlantMechanismSim(
            GEARBOX,
            GEAR_RATIO,
            volts(0.0, 1.0, 0.1).build(),
            Radians.of(-1),
            Radians.of(1),
            Radians.of(0));

    run(sim, 12.0, 1.0);
    assertEquals(1.0, sim.position().in(Radians));
    assertEquals(0.0, sim.velocity().in(RadiansPerSecond));

    run(sim, -12.0, 1.0);
    assertEquals(-1.0, sim.position().in(Radians));
    assertEquals(0.0, sim.velocity().in(RadiansPerSecond));

    PlantMechanismSim clamped =
        new PlantMechanismSim(
            GEARBOX,
            GEAR_RATIO,
            volts(0.0, 1.0, 0.1).build(),
            Radians.of(-1),
            Radians.of(1),
            Radians.of(5));
    assertEquals(1.0, clamped.position().in(Radians));
  }

  @Test
  public void testCurrentPlantMatchesItsVoltageEquivalent() {
    double resistance = GEARBOX.R;
    double backEmf = GEAR_RATIO / GEARBOX.Kv;
    PlantCharacterization<CurrentUnit> current =
        PlantCharacterization.current()
            .withKS(Amps.of(2.0))
            .withKV(Amps.per(RadiansPerSecond).ofNative(0.5))
            .withKA(Amps.per(RadiansPerSecondPerSecond).ofNative(0.1))
            .build();
    PlantCharacterization<VoltageUnit> voltage =
        volts(2.0 * resistance, 0.5 * resistance + backEmf, 0.1 * resistance).build();
    PlantMechanismSim currentSim = sim(current);
    PlantMechanismSim voltageSim = sim(voltage);

    run(currentSim, 4.0, 0.3);
    run(voltageSim, 4.0, 0.3);

    assertTrue(currentSim.velocity().in(RadiansPerSecond) > 0);
    assertEquals(
        voltageSim.velocity().in(RadiansPerSecond),
        currentSim.velocity().in(RadiansPerSecond),
        1e-9);
    assertEquals(voltageSim.position().in(Radians), currentSim.position().in(Radians), 1e-9);
  }

  @Test
  public void testCurrentDrawFollowsTheMotorModel() {
    PlantMechanismSim sim = sim(volts(0.0, 1.0, 0.1).build());

    run(sim, 6.0, 0.2);
    double velocity = sim.velocity().in(RadiansPerSecond);
    assertEquals(
        (6.0 - GEAR_RATIO * velocity / GEARBOX.Kv) / GEARBOX.R, sim.currentDraw().in(Amps), 1e-9);
  }

  @Test
  public void testExposesItsPlant() {
    PlantCharacterization<VoltageUnit> plant = volts(0.0, 1.0, 0.1).build();
    assertSame(plant, sim(plant).plant());
  }

  @Test
  public void shouldRejectBadGeometry() {
    PlantCharacterization<VoltageUnit> plant = volts(0.0, 1.0, 0.1).build();
    assertThrows(IllegalArgumentException.class, () -> new PlantMechanismSim(GEARBOX, 0, plant));
    assertThrows(
        IllegalArgumentException.class,
        () -> new PlantMechanismSim(GEARBOX, Double.POSITIVE_INFINITY, plant));
    assertThrows(
        IllegalArgumentException.class,
        () ->
            new PlantMechanismSim(
                GEARBOX, GEAR_RATIO, plant, Radians.of(1), Radians.of(0), Radians.of(0)));
    assertThrows(NullPointerException.class, () -> new PlantMechanismSim(null, GEAR_RATIO, plant));
    assertThrows(
        NullPointerException.class, () -> new PlantMechanismSim(GEARBOX, GEAR_RATIO, null));
  }
}
