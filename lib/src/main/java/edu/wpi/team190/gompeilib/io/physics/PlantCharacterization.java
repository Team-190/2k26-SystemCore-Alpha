package edu.wpi.team190.gompeilib.io.physics;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.io.motor.config.GravityType;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import java.util.Objects;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.PerUnit;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.Unit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Mass;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.units.measure.Per;

/**
 * How a mechanism responds to its motors, as the feedforward model it obeys at the gearbox output:
 *
 * <pre>u = kS·sign(ω) + kV·ω + kA·α + kG·g(θ)</pre>
 *
 * <p>where {@code u} is the effort (volts, or torque current per motor) and {@code g} is 1 for
 * {@link GravityType#CONSTANT} or {@code cos θ} for {@link GravityType#COSINE}, with θ = 0
 * horizontal. Simulation runs this model, so a mechanism tuned in sim behaves the same on the robot
 * as long as the plant matches what the robot measured.
 *
 * <p>The plant is fixed, unlike the tunable {@link edu.wpi.team190.gompeilib.control.Gains}: the
 * feedforward gains are the controller's guess at the plant, and keeping the two apart is what lets
 * the simulation show a wrong guess. Paste the characterization results into both.
 *
 * <p>Before the robot exists, start from {@link #ofInertia}, {@link #ofArm} or {@link #ofElevator},
 * which derive a voltage plant from the motors and CAD, then replace terms with measured ones
 * through {@link #toBuilder()} as they come in. The physics plants leave out friction and gearbox
 * losses, and the motors' rotors unless you list them as {@link Inertia} parts, which is what the
 * measured terms correct.
 *
 * @param <U> the effort unit: volts or amps
 */
public final class PlantCharacterization<U extends Unit> {
  private static final double GRAVITY = 9.80665;
  private static final double ROUNDING = 1e-9;

  private final U effortUnit;
  private final double kS;
  private final double kV;
  private final double kA;
  private final double kG;
  private final GravityType gravityType;

  private PlantCharacterization(Builder<U> builder) {
    effortUnit = builder.effortUnit;
    kS = builder.kS;
    kV = builder.kV;
    kA = builder.kA;
    kG = builder.kG;
    gravityType = builder.gravityType;
    if (kS < 0 || !Double.isFinite(kS)) {
      throw new IllegalArgumentException("kS must be non-negative and finite, was " + kS);
    }
    if (kV < 0 || !Double.isFinite(kV)) {
      throw new IllegalArgumentException("kV must be non-negative and finite, was " + kV);
    }
    if (!(kA > 0) || !Double.isFinite(kA)) {
      throw new IllegalArgumentException("kA must be positive and finite, was " + kA);
    }
    if (!Double.isFinite(kG)) {
      throw new IllegalArgumentException("kG must be finite, was " + kG);
    }
  }

  /**
   * Starts a plant whose effort is voltage.
   *
   * @return the builder
   */
  public static Builder<VoltageUnit> voltage() {
    return new Builder<>(Volts);
  }

  /**
   * Starts a plant whose effort is torque current, per motor.
   *
   * @return the builder
   */
  public static Builder<CurrentUnit> current() {
    return new Builder<>(Amps);
  }

  /**
   * Derives a voltage plant for a geared inertia with no external load, such as a roller, flywheel,
   * or turret: {@code kV = G/Kv} and {@code kA = R·J/(G·Kt)}.
   *
   * @param gearbox the motors driving the mechanism
   * @param gearRatio rotor rotations per output rotation
   * @param momentOfInertia the mechanism's moment of inertia about its output
   * @return the builder, to override terms before building
   */
  public static Builder<VoltageUnit> ofInertia(
      DCMotor gearbox, double gearRatio, MomentOfInertia momentOfInertia) {
    double inertia = momentOfInertia.in(KilogramSquareMeters);
    return voltage()
        .withKV(Volts.per(RadiansPerSecond).ofNative(gearRatio / gearbox.Kv))
        .withKA(
            Volts.per(RadiansPerSecondPerSecond)
                .ofNative(gearbox.R * inertia / (gearRatio * gearbox.Kt)));
  }

  /**
   * Derives a voltage plant for a mechanism made of several rotating parts, each at its own speed,
   * such as a flywheel's drum, a geared-up inertia disk, and the motors' rotors. Their inertias add
   * at the output as {@link Inertia#total}.
   *
   * @param gearbox the motors driving the mechanism
   * @param gearRatio rotor rotations per output rotation
   * @param parts the rotating parts
   * @return the builder, to override terms before building
   */
  public static Builder<VoltageUnit> ofInertia(
      DCMotor gearbox, double gearRatio, Inertia... parts) {
    return ofInertia(gearbox, gearRatio, Inertia.total(parts));
  }

  /**
   * Derives a voltage plant for an arm, adding {@code kG = R·m·g·L/(G·Kt)} applied as {@link
   * GravityType#COSINE}.
   *
   * @param gearbox the motors driving the arm
   * @param gearRatio rotor rotations per arm rotation
   * @param mass the arm's mass
   * @param centerOfMass the distance from the joint to the arm's center of mass
   * @param parts the rotating parts: the arm about its joint, at the output, and anything geared to
   *     it, such as the motors' rotors
   * @return the builder, to override terms before building
   */
  public static Builder<VoltageUnit> ofArm(
      DCMotor gearbox, double gearRatio, Mass mass, Distance centerOfMass, Inertia... parts) {
    double holdingTorque = mass.in(Kilograms) * GRAVITY * centerOfMass.in(Meters);
    return ofInertia(gearbox, gearRatio, parts)
        .withKG(Volts.of(gearbox.R * holdingTorque / (gearRatio * gearbox.Kt)))
        .withGravityType(GravityType.COSINE);
  }

  /**
   * Derives a voltage plant for an elevator, measured at the drum: the carriage reflects an inertia
   * of {@code m·r²}, the rotating parts add theirs, and {@code kG = R·m·g·r/(G·Kt)} applied as
   * {@link GravityType#CONSTANT}.
   *
   * @param gearbox the motors driving the carriage
   * @param gearRatio rotor rotations per drum rotation
   * @param carriageMass the carriage's mass
   * @param drumRadius the drum's radius
   * @param parts any rotating parts besides the carriage, such as the drum, pulleys, and the
   *     motors' rotors
   * @return the builder, to override terms before building
   */
  public static Builder<VoltageUnit> ofElevator(
      DCMotor gearbox, double gearRatio, Mass carriageMass, Distance drumRadius, Inertia... parts) {
    double mass = carriageMass.in(Kilograms);
    double radius = drumRadius.in(Meters);
    double carriage = mass * radius * radius;
    double rotating = Inertia.total(parts).in(KilogramSquareMeters);
    return ofInertia(gearbox, gearRatio, KilogramSquareMeters.of(carriage + rotating))
        .withKG(Volts.of(gearbox.R * mass * GRAVITY * radius / (gearRatio * gearbox.Kt)))
        .withGravityType(GravityType.CONSTANT);
  }

  /**
   * Returns this plant with torque current as its effort. The motors turn voltage into current as
   * {@code V = I·R + G·ω/Kv} per motor, so every term divides by one motor's resistance, and the
   * back-EMF leaves kV: what remains is viscous friction.
   *
   * @param gearbox the motors driving the mechanism
   * @param gearRatio rotor rotations per output rotation
   * @param motorCount how many motors {@code gearbox} models
   * @return the current plant, or this plant if it already is one
   * @throws IllegalArgumentException if the voltage kV is below the motors' back-EMF alone
   */
  @SuppressWarnings("unchecked")
  public PlantCharacterization<CurrentUnit> toCurrent(
      DCMotor gearbox, double gearRatio, int motorCount) {
    if (effortUnit instanceof CurrentUnit) {
      return (PlantCharacterization<CurrentUnit>) this;
    }
    double resistance = motorResistance(gearbox, motorCount);
    double backEmf = gearRatio / gearbox.Kv;
    // A physics plant's kV is exactly the back-EMF; don't let rounding make it negative
    double viscous = Math.abs(kV - backEmf) <= ROUNDING * backEmf ? 0.0 : kV - backEmf;
    return convert(current(), viscous / resistance, 1.0 / resistance);
  }

  /**
   * Returns this plant with voltage as its effort, the inverse of {@link #toCurrent}.
   *
   * @param gearbox the motors driving the mechanism
   * @param gearRatio rotor rotations per output rotation
   * @param motorCount how many motors {@code gearbox} models
   * @return the voltage plant, or this plant if it already is one
   */
  @SuppressWarnings("unchecked")
  public PlantCharacterization<VoltageUnit> toVoltage(
      DCMotor gearbox, double gearRatio, int motorCount) {
    if (effortUnit instanceof VoltageUnit) {
      return (PlantCharacterization<VoltageUnit>) this;
    }
    double resistance = motorResistance(gearbox, motorCount);
    return convert(voltage(), kV * resistance + gearRatio / gearbox.Kv, resistance);
  }

  /**
   * Returns this plant with the given effort.
   *
   * @param effort {@link OutputType#VOLTAGE} or {@link OutputType#CURRENT}
   * @param gearbox the motors driving the mechanism
   * @param gearRatio rotor rotations per output rotation
   * @param motorCount how many motors {@code gearbox} models
   * @return the plant in that effort
   * @throws IllegalArgumentException for duty cycle
   */
  public PlantCharacterization<?> in(
      OutputType effort, DCMotor gearbox, double gearRatio, int motorCount) {
    return switch (effort) {
      case VOLTAGE -> toVoltage(gearbox, gearRatio, motorCount);
      case CURRENT -> toCurrent(gearbox, gearRatio, motorCount);
      case DUTY_CYCLE ->
          throw new IllegalArgumentException("A plant's effort is volts or amps, not duty cycle");
    };
  }

  private static double motorResistance(DCMotor gearbox, int motorCount) {
    if (motorCount < 1) {
      throw new IllegalArgumentException("Motor count must be positive, was " + motorCount);
    }
    return gearbox.R * motorCount;
  }

  /** Scales kS, kA and kG by {@code scale} into a new builder, with the converted kV. */
  private <T extends Unit> PlantCharacterization<T> convert(
      Builder<T> builder, double convertedKV, double scale) {
    builder.kS = kS * scale;
    builder.kV = convertedKV;
    builder.kA = kA * scale;
    builder.kG = kG * scale;
    builder.gravityType = gravityType;
    return builder.build();
  }

  /**
   * Returns a builder holding this plant, to replace some of its terms.
   *
   * @return the builder
   */
  public Builder<U> toBuilder() {
    Builder<U> builder = new Builder<>(effortUnit);
    builder.kS = kS;
    builder.kV = kV;
    builder.kA = kA;
    builder.kG = kG;
    builder.gravityType = gravityType;
    return builder;
  }

  /**
   * Returns the effort unit.
   *
   * @return volts or amps
   */
  public U effortUnit() {
    return effortUnit;
  }

  /**
   * Returns what the effort is.
   *
   * @return {@link OutputType#VOLTAGE} or {@link OutputType#CURRENT}
   */
  public OutputType effort() {
    return effortUnit instanceof VoltageUnit ? OutputType.VOLTAGE : OutputType.CURRENT;
  }

  /**
   * Returns the static friction.
   *
   * @return kS, in base effort units
   */
  public double kS() {
    return kS;
  }

  /**
   * Returns the effort per output velocity.
   *
   * @return kV, in base effort units per radian per second
   */
  public double kV() {
    return kV;
  }

  /**
   * Returns the effort per output acceleration.
   *
   * @return kA, in base effort units per radian per second squared
   */
  public double kA() {
    return kA;
  }

  /**
   * Returns the effort that holds the mechanism against gravity at full load.
   *
   * @return kG, in base effort units
   */
  public double kG() {
    return kG;
  }

  /**
   * Returns how gravity's load varies with position.
   *
   * @return the gravity type
   */
  public GravityType gravityType() {
    return gravityType;
  }

  /**
   * Returns the gravity load's fraction of {@code kG} at a position.
   *
   * @param radians the position at the gearbox output
   * @return 1 for a constant load, or the cosine of the position
   */
  public double gravityFactor(double radians) {
    return gravityType == GravityType.COSINE ? Math.cos(radians) : 1.0;
  }

  @Override
  public boolean equals(Object other) {
    return other instanceof PlantCharacterization<?> plant
        && effortUnit.equals(plant.effortUnit)
        && kS == plant.kS
        && kV == plant.kV
        && kA == plant.kA
        && kG == plant.kG
        && gravityType == plant.gravityType;
  }

  @Override
  public int hashCode() {
    return Objects.hash(effortUnit, kS, kV, kA, kG, gravityType);
  }

  @Override
  public String toString() {
    String unit = effortUnit.symbol();
    return String.format(
        "PlantCharacterization[kS=%.6g %s, kV=%.6g %s/(rad/s), kA=%.6g %s/(rad/s²), kG=%.6g %s"
            + " (%s)]",
        kS, unit, kV, unit, kA, unit, kG, unit, gravityType);
  }

  /**
   * Builds a {@link PlantCharacterization}. Terms left unset are zero, except {@code kA}, which
   * must be set; gravity defaults to {@link GravityType#CONSTANT}.
   *
   * @param <U> the effort unit
   */
  public static final class Builder<U extends Unit> {
    private final U effortUnit;
    private double kS = 0.0;
    private double kV = 0.0;
    private double kA = 0.0;
    private double kG = 0.0;
    private GravityType gravityType = GravityType.CONSTANT;

    private Builder(U effortUnit) {
      this.effortUnit = effortUnit;
    }

    public Builder<U> withKS(Measure<U> kS) {
      this.kS = Objects.requireNonNull(kS, "kS").baseUnitMagnitude();
      return this;
    }

    public Builder<U> withKV(Per<U, AngularVelocityUnit> kV) {
      this.kV = Objects.requireNonNull(kV, "kV").baseUnitMagnitude();
      return this;
    }

    public Builder<U> withKA(Per<U, ? extends PerUnit<AngularVelocityUnit, TimeUnit>> kA) {
      this.kA = Objects.requireNonNull(kA, "kA").baseUnitMagnitude();
      return this;
    }

    public Builder<U> withKG(Measure<U> kG) {
      this.kG = Objects.requireNonNull(kG, "kG").baseUnitMagnitude();
      return this;
    }

    public Builder<U> withGravityType(GravityType gravityType) {
      this.gravityType = Objects.requireNonNull(gravityType, "gravityType");
      return this;
    }

    /**
     * Builds the plant.
     *
     * @return the plant
     * @throws IllegalArgumentException if {@code kA} isn't positive, or {@code kS} or {@code kV} is
     *     negative
     */
    public PlantCharacterization<U> build() {
      return new PlantCharacterization<>(this);
    }
  }
}
