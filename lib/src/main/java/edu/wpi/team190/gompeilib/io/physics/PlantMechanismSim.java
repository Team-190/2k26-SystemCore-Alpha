package edu.wpi.team190.gompeilib.io.physics;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import java.util.Objects;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;

/**
 * A mechanism that obeys its {@link PlantCharacterization}: the applied voltage moves it exactly as
 * the plant's feedforward model says, including static friction and gravity. This is the only
 * physics model, so every simulated mechanism, whichever motor IO drives it, responds the way the
 * robot measured.
 *
 * <p>A voltage plant takes the voltage as its effort. A current plant takes the torque current that
 * voltage drives through the motors, {@code I = (V − G·ω/Kv)/(R·n)} per motor, so a mechanism
 * characterized in torque current behaves the same under a torque-current loop.
 *
 * <p>Each step solves the linear part exactly for a constant input, so stiff mechanisms stay stable
 * at any time step. Static friction holds the mechanism still while the net effort is within {@code
 * kS}, and stops it rather than reversing it. Optional hard stops clamp the position.
 *
 * <p>Not modeled: backlash, battery sag, CAN latency, sensor noise.
 */
public class PlantMechanismSim extends AbstractMechanismSim {
  /**
   * Below this speed, in radians per second, the mechanism is at rest and static friction holds.
   */
  private static final double REST_VELOCITY = 1e-6;

  private final double gearRatio;
  private final PlantCharacterization<?> plant;
  private final double minRadians;
  private final double maxRadians;
  private double positionRadians;
  private double velocityRadPerSec = 0.0;
  private double volts = 0.0;

  /**
   * Creates an unbounded mechanism starting at zero.
   *
   * @param gearbox the motors driving the mechanism
   * @param gearRatio rotor rotations per output rotation
   * @param plant the plant
   */
  public PlantMechanismSim(DCMotor gearbox, double gearRatio, PlantCharacterization<?> plant) {
    this(
        gearbox,
        gearRatio,
        plant,
        Radians.of(Double.NEGATIVE_INFINITY),
        Radians.of(Double.POSITIVE_INFINITY),
        Radians.of(0.0));
  }

  /**
   * Creates a mechanism between hard stops.
   *
   * @param gearbox the motors driving the mechanism
   * @param gearRatio rotor rotations per output rotation
   * @param plant the plant
   * @param min the lowest position at the gearbox output
   * @param max the highest position at the gearbox output
   * @param start the starting position
   * @throws IllegalArgumentException if the gear ratio isn't positive, or the stops are out of
   *     order
   */
  public PlantMechanismSim(
      DCMotor gearbox,
      double gearRatio,
      PlantCharacterization<?> plant,
      Angle min,
      Angle max,
      Angle start) {
    super(Objects.requireNonNull(gearbox, "gearbox"));
    if (!(gearRatio > 0) || !Double.isFinite(gearRatio)) {
      throw new IllegalArgumentException(
          "Gear ratio must be positive and finite, was " + gearRatio);
    }
    this.gearRatio = gearRatio;
    this.plant = Objects.requireNonNull(plant, "plant");
    minRadians = min.in(Radians);
    maxRadians = max.in(Radians);
    if (minRadians > maxRadians) {
      throw new IllegalArgumentException("Min " + min + " is above max " + max);
    }
    positionRadians = Math.clamp(start.in(Radians), minRadians, maxRadians);
  }

  /**
   * Returns the plant the mechanism obeys.
   *
   * @return the plant
   */
  public PlantCharacterization<?> plant() {
    return plant;
  }

  @Override
  protected void applyVoltage(double volts) {
    this.volts = volts;
  }

  @Override
  protected void step(double seconds) {
    // Effort = scale·V − backEmf·ω: a current plant sees the torque current the voltage drives
    double scale = 1.0;
    double backEmf = 0.0;
    if (plant.effort() == OutputType.CURRENT) {
      double resistance = gearbox().R * motorCount();
      scale = 1.0 / resistance;
      backEmf = gearRatio / (gearbox().Kv * resistance);
    }
    double drive = scale * volts - plant.kG() * plant.gravityFactor(positionRadians);

    double direction;
    if (Math.abs(velocityRadPerSec) < REST_VELOCITY) {
      if (Math.abs(drive) <= plant.kS()) {
        velocityRadPerSec = 0.0;
        return;
      }
      direction = Math.signum(drive);
    } else {
      direction = Math.signum(velocityRadPerSec);
    }

    // kA·α = force − damping·ω, solved exactly for this step
    double force = drive - plant.kS() * direction;
    double damping = plant.kV() + backEmf;
    double start = velocityRadPerSec;
    double velocity;
    double travel;
    if (damping > 0) {
      double steadyState = force / damping;
      double timeConstant = plant.kA() / damping;
      double decay = Math.exp(-seconds / timeConstant);
      velocity = steadyState + (start - steadyState) * decay;
      travel = steadyState * seconds + (start - steadyState) * timeConstant * (1.0 - decay);
    } else {
      double acceleration = force / plant.kA();
      velocity = start + acceleration * seconds;
      travel = start * seconds + 0.5 * acceleration * seconds * seconds;
    }
    // Friction stops the mechanism; it never pushes it backward
    if (velocity * direction < 0) {
      velocity = 0.0;
    }

    double position = positionRadians + travel;
    if (position <= minRadians) {
      position = minRadians;
      velocity = Math.max(velocity, 0.0);
    } else if (position >= maxRadians) {
      position = maxRadians;
      velocity = Math.min(velocity, 0.0);
    }
    positionRadians = position;
    velocityRadPerSec = velocity;
  }

  @Override
  public Angle position() {
    return Radians.of(positionRadians);
  }

  @Override
  public AngularVelocity velocity() {
    return RadiansPerSecond.of(velocityRadPerSec);
  }

  @Override
  public Current currentDraw() {
    return Amps.of((volts - gearRatio * velocityRadPerSec / gearbox().Kv) / gearbox().R);
  }
}
