package edu.wpi.team190.gompeilib.subsystems.arm;

import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import java.util.Optional;
import lombok.Builder;
import lombok.NonNull;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.units.measure.Voltage;

@Builder(setterPrefix = "with")
public class ArmConstants {
  /**
   * The leader motor and its followers. Its gear ratio is rotor rotations per arm rotation; use
   * {@link edu.wpi.team190.gompeilib.io.motor.config.GravityType#COSINE} slots so {@code kG} holds
   * the arm against gravity, and {@link MotorConfig#continuousWrap} for an arm that spins freely.
   * The arm adds profile constraints from {@link #constraints}. Set {@link MotorConfig#softLimits}
   * to keep closed-loop control inside the travel range.
   */
  @NonNull public final MotorConfig motor;

  /** The travel range and physical properties. */
  @NonNull public final ArmParameters armParameters;

  /** The goal tolerance and profile limits. */
  @NonNull public final AngularPositionConstraints constraints;

  /** How far one increment or decrement moves the voltage goal. */
  @NonNull public final Voltage voltageOffsetStep;

  /** How far one increment or decrement moves the position goal. */
  @NonNull public final Rotation2d positionOffsetStep;

  /** How the arm finds its position against a hard stop, or empty if it doesn't home. */
  @NonNull @Builder.Default public final Optional<Homing> homing = Optional.empty();

  /**
   * Current-based homing: the arm drives into a hard stop until the leader's torque current reaches
   * a threshold, then takes that stop's angle as its position.
   *
   * @param voltage the voltage that drives the arm into the hard stop; its sign picks the direction
   * @param currentThreshold the torque current that means the arm has reached the hard stop
   * @param currentEpsilon how close to {@code currentThreshold} counts as reaching it
   * @param position the arm's angle at the hard stop
   */
  @Builder(setterPrefix = "with")
  public record Homing(
      @NonNull Voltage voltage,
      @NonNull Current currentThreshold,
      @NonNull Current currentEpsilon,
      @NonNull Rotation2d position) {}

  /**
   * The arm's travel and physical properties.
   *
   * @param armGearbox the motors driving the arm, for simulation
   * @param minAngle the lowest angle
   * @param maxAngle the highest angle
   * @param length the arm's length, for simulation
   * @param momentOfInertia the arm's moment of inertia, for simulation
   * @param simulateGravity whether gravity pulls the arm down, for simulation
   */
  @Builder(setterPrefix = "with")
  public record ArmParameters(
      @NonNull DCMotor armGearbox,
      @NonNull Rotation2d minAngle,
      @NonNull Rotation2d maxAngle,
      @NonNull Distance length,
      @NonNull MomentOfInertia momentOfInertia,
      @NonNull Boolean simulateGravity) {}
}
