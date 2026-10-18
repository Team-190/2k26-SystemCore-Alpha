package edu.wpi.team190.gompeilib.robot.mechanisms.extension;

import edu.wpi.team190.gompeilib.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.robot.mechanisms.CurrentHoming;
import edu.wpi.team190.gompeilib.sysid.SysIdConfig;
import java.util.Optional;
import lombok.Builder;
import lombok.NonNull;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.VoltageUnit;
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

  /** How the extension finds its position against a hard stop, or empty if it doesn't home. */
  @NonNull @Builder.Default
  public final Optional<CurrentHoming.Config<DistanceUnit>> homing = Optional.empty();

  /** The travel range and physical properties. */
  @NonNull public final ExtensionParameters extensionParameters;

  /**
   * The goal tolerance, profile limits, and how far short of the travel limits a SysId leg stops.
   */
  @NonNull public final LinearConstraints constraints;

  /** How the extension is characterized in voltage. */
  @NonNull public final SysIdConfig<VoltageUnit> sysId;

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
