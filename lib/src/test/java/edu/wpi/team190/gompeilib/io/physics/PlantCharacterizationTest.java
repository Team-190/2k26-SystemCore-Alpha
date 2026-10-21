package edu.wpi.team190.gompeilib.io.physics;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.io.motor.config.GravityType;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.VoltageUnit;

public class PlantCharacterizationTest {
  private static final double EPSILON = 1e-12;
  private static final DCMotor GEARBOX = DCMotor.getKrakenX60(2);

  private static PlantCharacterization<VoltageUnit> plant() {
    return PlantCharacterization.voltage()
        .withKS(Volts.of(0.25))
        .withKV(Volts.per(RotationsPerSecond).ofNative(2 * Math.PI))
        .withKA(Volts.per(RotationsPerSecondPerSecond).ofNative(Math.PI))
        .withKG(Volts.of(0.75))
        .withGravityType(GravityType.COSINE)
        .build();
  }

  @Test
  public void testBuilderStoresEveryTermInBaseUnits() {
    PlantCharacterization<VoltageUnit> plant = plant();

    assertEquals(Volts, plant.effortUnit());
    assertEquals(OutputType.VOLTAGE, plant.effort());
    assertEquals(0.25, plant.kS(), EPSILON);
    assertEquals(1.0, plant.kV(), EPSILON);
    assertEquals(0.5, plant.kA(), EPSILON);
    assertEquals(0.75, plant.kG(), EPSILON);
    assertEquals(GravityType.COSINE, plant.gravityType());
  }

  @Test
  public void testCurrentPlant() {
    PlantCharacterization<CurrentUnit> plant =
        PlantCharacterization.current()
            .withKA(Amps.per(RadiansPerSecondPerSecond).ofNative(0.1))
            .build();

    assertEquals(Amps, plant.effortUnit());
    assertEquals(OutputType.CURRENT, plant.effort());
    assertEquals(0.0, plant.kS());
    assertEquals(0.0, plant.kV());
    assertEquals(0.1, plant.kA(), EPSILON);
    assertEquals(0.0, plant.kG());
    assertEquals(GravityType.CONSTANT, plant.gravityType());
  }

  @Test
  public void testGravityFactor() {
    assertEquals(0.5, plant().gravityFactor(Math.PI / 3), EPSILON);
    PlantCharacterization<VoltageUnit> constant =
        plant().toBuilder().withGravityType(GravityType.CONSTANT).build();
    assertEquals(1.0, constant.gravityFactor(Math.PI / 3));
  }

  @Test
  public void testOfInertiaUsesTheMotorModel() {
    PlantCharacterization<VoltageUnit> plant =
        PlantCharacterization.ofInertia(GEARBOX, 10.0, KilogramSquareMeters.of(0.2)).build();

    assertEquals(10.0 / GEARBOX.Kv, plant.kV(), EPSILON);
    assertEquals(GEARBOX.R * 0.2 / (10.0 * GEARBOX.Kt), plant.kA(), EPSILON);
    assertEquals(0.0, plant.kS());
    assertEquals(0.0, plant.kG());
  }

  @Test
  public void testOfArmAddsCosineGravity() {
    PlantCharacterization<VoltageUnit> plant =
        PlantCharacterization.ofArm(
                GEARBOX,
                50.0,
                Kilograms.of(2),
                Meters.of(0.3),
                Inertia.atOutput(KilogramSquareMeters.of(0.1)))
            .build();

    assertEquals(GEARBOX.R * 2 * 9.80665 * 0.3 / (50.0 * GEARBOX.Kt), plant.kG(), EPSILON);
    assertEquals(GravityType.COSINE, plant.gravityType());
    assertEquals(GEARBOX.R * 0.1 / (50.0 * GEARBOX.Kt), plant.kA(), EPSILON);
  }

  @Test
  public void testOfElevatorReflectsTheCarriageThroughTheDrum() {
    PlantCharacterization<VoltageUnit> plant =
        PlantCharacterization.ofElevator(GEARBOX, 5.0, Kilograms.of(4), Meters.of(0.02)).build();

    assertEquals(GEARBOX.R * 4 * 0.02 * 0.02 / (5.0 * GEARBOX.Kt), plant.kA(), EPSILON);
    assertEquals(GEARBOX.R * 4 * 9.80665 * 0.02 / (5.0 * GEARBOX.Kt), plant.kG(), EPSILON);
    assertEquals(GravityType.CONSTANT, plant.gravityType());
  }

  @Test
  public void testSeveralPartsAddAtTheOutput() {
    Inertia drum = Inertia.atOutput(KilogramSquareMeters.of(0.09));
    Inertia disk = Inertia.geared(KilogramSquareMeters.of(0.01), 2.0);
    Inertia rotors = Inertia.geared(KilogramSquareMeters.of(0.0001), 10.0);

    assertEquals(
        PlantCharacterization.ofInertia(GEARBOX, 10.0, KilogramSquareMeters.of(0.09 + 0.04 + 0.01))
            .build(),
        PlantCharacterization.ofInertia(GEARBOX, 10.0, drum, disk, rotors).build());

    // The drum and its rotors add to the carriage's m·r²
    PlantCharacterization<VoltageUnit> elevator =
        PlantCharacterization.ofElevator(
                GEARBOX, 5.0, Kilograms.of(4), Meters.of(0.02), drum, rotors)
            .build();
    assertEquals(
        GEARBOX.R * (4 * 0.02 * 0.02 + 0.09 + 0.01) / (5.0 * GEARBOX.Kt), elevator.kA(), EPSILON);
  }

  @Test
  public void testToBuilderOverridesSomeTerms() {
    PlantCharacterization<VoltageUnit> measured =
        PlantCharacterization.ofInertia(GEARBOX, 10.0, KilogramSquareMeters.of(0.2))
            .withKS(Volts.of(0.3))
            .build()
            .toBuilder()
            .withKA(Volts.per(RadiansPerSecondPerSecond).ofNative(0.05))
            .build();

    assertEquals(0.3, measured.kS(), EPSILON);
    assertEquals(10.0 / GEARBOX.Kv, measured.kV(), EPSILON);
    assertEquals(0.05, measured.kA(), EPSILON);
    assertEquals(plant(), plant().toBuilder().build());
  }

  @Test
  public void testConvertsBetweenVoltageAndCurrent() {
    // Measured: kV above the back-EMF, so viscous friction remains in current
    double backEmf = 10.0 / GEARBOX.Kv;
    double resistance = GEARBOX.R * 2;
    PlantCharacterization<VoltageUnit> voltage =
        PlantCharacterization.voltage()
            .withKS(Volts.of(0.4))
            .withKV(Volts.per(RadiansPerSecond).ofNative(backEmf + 0.1))
            .withKA(Volts.per(RadiansPerSecondPerSecond).ofNative(0.05))
            .withKG(Volts.of(0.2))
            .withGravityType(GravityType.COSINE)
            .build();

    PlantCharacterization<CurrentUnit> current = voltage.toCurrent(GEARBOX, 10.0, 2);
    assertEquals(0.4 / resistance, current.kS(), EPSILON);
    assertEquals(0.1 / resistance, current.kV(), EPSILON);
    assertEquals(0.05 / resistance, current.kA(), EPSILON);
    assertEquals(0.2 / resistance, current.kG(), EPSILON);
    assertEquals(GravityType.COSINE, current.gravityType());

    PlantCharacterization<VoltageUnit> back = current.toVoltage(GEARBOX, 10.0, 2);
    assertEquals(voltage.kS(), back.kS(), EPSILON);
    assertEquals(voltage.kV(), back.kV(), EPSILON);
    assertEquals(voltage.kA(), back.kA(), EPSILON);
    assertEquals(voltage.kG(), back.kG(), EPSILON);

    assertSame(voltage, voltage.toVoltage(GEARBOX, 10.0, 2));
    assertSame(current, current.toCurrent(GEARBOX, 10.0, 2));
    assertSame(voltage, voltage.in(OutputType.VOLTAGE, GEARBOX, 10.0, 2));
    assertEquals(current, voltage.in(OutputType.CURRENT, GEARBOX, 10.0, 2));
  }

  @Test
  public void testPhysicsPlantHasNoViscousFrictionInCurrent() {
    PlantCharacterization<CurrentUnit> current =
        PlantCharacterization.ofInertia(GEARBOX, 7.3, KilogramSquareMeters.of(0.2))
            .build()
            .toCurrent(GEARBOX, 7.3, 2);

    assertEquals(0.0, current.kV());
    // Torque per motor amp: J·α = 2·Kt·G·I
    assertEquals(0.2 / (2 * GEARBOX.Kt * 7.3), current.kA(), EPSILON);
  }

  @Test
  public void shouldRejectConversionsThatCantBeMade() {
    PlantCharacterization<VoltageUnit> plant = plant();
    assertThrows(IllegalArgumentException.class, () -> plant.toCurrent(GEARBOX, 10.0, 0));
    assertThrows(
        IllegalArgumentException.class, () -> plant.in(OutputType.DUTY_CYCLE, GEARBOX, 10.0, 2));
    // A voltage kV below the back-EMF alone would need negative friction
    assertThrows(IllegalArgumentException.class, () -> plant.toCurrent(GEARBOX, 100.0, 2));
  }

  @Test
  public void shouldRejectTermsTheModelCantRun() {
    assertThrows(
        IllegalArgumentException.class, () -> plant().toBuilder().withKS(Volts.of(-0.1)).build());
    assertThrows(
        IllegalArgumentException.class,
        () -> plant().toBuilder().withKS(Volts.of(Double.POSITIVE_INFINITY)).build());
    assertThrows(
        IllegalArgumentException.class,
        () -> plant().toBuilder().withKV(Volts.per(RadiansPerSecond).ofNative(-1)).build());
    assertThrows(
        IllegalArgumentException.class,
        () -> plant().toBuilder().withKV(Volts.per(RadiansPerSecond).ofNative(Double.NaN)).build());
    assertThrows(
        IllegalArgumentException.class,
        () -> plant().toBuilder().withKA(Volts.per(RadiansPerSecondPerSecond).ofNative(0)).build());
    assertThrows(
        IllegalArgumentException.class,
        () ->
            plant().toBuilder()
                .withKA(Volts.per(RadiansPerSecondPerSecond).ofNative(Double.POSITIVE_INFINITY))
                .build());
    assertThrows(
        IllegalArgumentException.class,
        () -> plant().toBuilder().withKG(Volts.of(Double.NaN)).build());
    assertThrows(IllegalArgumentException.class, () -> PlantCharacterization.voltage().build());
    assertThrows(NullPointerException.class, () -> PlantCharacterization.voltage().withKS(null));
    assertThrows(NullPointerException.class, () -> PlantCharacterization.voltage().withKV(null));
    assertThrows(NullPointerException.class, () -> PlantCharacterization.voltage().withKA(null));
    assertThrows(NullPointerException.class, () -> PlantCharacterization.voltage().withKG(null));
    assertThrows(
        NullPointerException.class, () -> PlantCharacterization.voltage().withGravityType(null));
  }

  @Test
  public void testEqualsComparesEveryTerm() {
    PlantCharacterization<VoltageUnit> plant = plant();

    assertEquals(plant, plant());
    assertEquals(plant.hashCode(), plant().hashCode());
    assertNotEquals(plant, "plant");
    assertNotEquals(
        plant,
        PlantCharacterization.current()
            .withKS(Amps.of(0.25))
            .withKV(Amps.per(RadiansPerSecond).ofNative(1.0))
            .withKA(Amps.per(RadiansPerSecondPerSecond).ofNative(0.5))
            .withKG(Amps.of(0.75))
            .withGravityType(GravityType.COSINE)
            .build());
    assertNotEquals(plant, plant.toBuilder().withKS(Volts.of(0.3)).build());
    assertNotEquals(
        plant, plant.toBuilder().withKV(Volts.per(RadiansPerSecond).ofNative(1.1)).build());
    assertNotEquals(
        plant,
        plant.toBuilder().withKA(Volts.per(RadiansPerSecondPerSecond).ofNative(0.6)).build());
    assertNotEquals(plant, plant.toBuilder().withKG(Volts.of(0.8)).build());
    assertNotEquals(plant, plant.toBuilder().withGravityType(GravityType.CONSTANT).build());
  }

  @Test
  public void testToStringNamesEveryTerm() {
    assertEquals(
        "PlantCharacterization[kS=0.250000 V, kV=1.00000 V/(rad/s), kA=0.500000 V/(rad/s²),"
            + " kG=0.750000 V (COSINE)]",
        plant().toString());
  }
}
