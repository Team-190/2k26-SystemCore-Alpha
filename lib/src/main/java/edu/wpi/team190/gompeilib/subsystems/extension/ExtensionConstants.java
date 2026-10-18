package edu.wpi.team190.gompeilib.subsystems.extension;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import lombok.Builder;
import lombok.NonNull;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Mass;
import org.wpilib.units.measure.Voltage;

@Builder(setterPrefix = "with")
public class ExtensionConstants {
  /**
   * The leader motor and its followers. Its gear ratio is rotor rotations per drum rotation, and
   * its gains are per drum rotation. The extension adds soft limits from the travel range and
   * profile constraints from {@link #constraints}.
   */
  @NonNull public final MotorConfig motor;

  /** The radius of the drum that turns rotation into travel. */
  @NonNull public final Distance drumRadius;

  /** The voltage the zeroing routine drives toward the hard stop. */
  @Builder.Default public final Voltage zeroVoltage = Volts.of(1.0);

  /** The torque current that means the extension has reached the hard stop. */
  @Builder.Default public final Current zeroCurrentThreshold = Amps.of(-20.0);

  /** How close to {@link #zeroCurrentThreshold} counts as reaching it. */
  @Builder.Default public final Current zeroCurrentEpsilon = Amps.of(5.0);

  /** The travel range and physical properties. */
  @NonNull public final ExtensionParameters extensionParameters;

  /** The goal tolerance and profile limits. */
  @NonNull public final LinearConstraints constraints;

  /** How far one increment or decrement moves the voltage goal. */
  @NonNull public final Voltage voltageOffsetStep;

  /** How far one increment or decrement moves the position goal. */
  @NonNull public final Distance heightOffsetStep;

  /**
   * The extension's travel and physical properties.
   *
   * @param carriageGearbox the motors driving the carriage, for simulation
   * @param carriageMass the carriage's mass, for simulation
   * @param minLength the shortest extension
   * @param maxLength the longest extension
   * @param simulateGravity whether gravity pulls the carriage back, for simulation
   */
  @Builder(setterPrefix = "with")
  public record ExtensionParameters(
      @NonNull DCMotor carriageGearbox,
      @NonNull Mass carriageMass,
      @NonNull Distance minLength,
      @NonNull Distance maxLength,
      @NonNull Boolean simulateGravity) {}
}
