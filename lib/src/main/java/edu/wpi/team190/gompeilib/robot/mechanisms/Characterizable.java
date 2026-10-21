package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Volts;

import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConstraint;
import edu.wpi.team190.gompeilib.characterization.CharacterizationRoutine;
import edu.wpi.team190.gompeilib.io.motor.config.GravityType;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import org.wpilib.command2.Command;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.measure.Distance;

/**
 * A mechanism that can be characterized with the shared {@link CharacterizationRoutine}. A
 * positional mechanism overrides {@link #characterizationConstraint()} with its travel limits; a
 * continuous one keeps the default and runs every leg to its timeout. A mechanism gravity loads
 * overrides {@link #characterizationGravity()} so the fit includes {@code kG}, and one that turns a
 * drum overrides {@link #characterizationDrumRadius()} so the report prints gains per meter.
 */
public interface Characterizable extends Mechanism {
  /**
   * Returns the travel the routine must keep the mechanism inside.
   *
   * @return the constraint, or empty for a continuous mechanism
   */
  default Optional<CharacterizationConstraint<?>> characterizationConstraint() {
    return Optional.empty();
  }

  /**
   * Returns how gravity loads the mechanism.
   *
   * @return the gravity type, or empty if gravity doesn't load it
   */
  default Optional<GravityType> characterizationGravity() {
    return Optional.empty();
  }

  /**
   * Returns the drum that turns the mechanism's rotation into travel.
   *
   * @return the drum's radius, or empty for an angular mechanism
   */
  default Optional<Distance> characterizationDrumRadius() {
    return Optional.empty();
  }

  /**
   * Returns this mechanism as a routine target, for characterizing several mechanisms together.
   *
   * @param request how an output becomes a request to the motors, such as {@link
   *     VoltageControl#request(Measure)}
   * @param <U> the output unit
   * @return the target
   */
  default <U extends Unit> CharacterizationRoutine.Target<U> characterizationTarget(
      Function<Measure<U>, MotorRequest> request) {
    return new CharacterizationRoutine.MechanismTarget<>(
        core(),
        request,
        characterizationConstraint(),
        characterizationGravity(),
        characterizationDrumRadius());
  }

  /**
   * Returns this mechanism as a routine target driven open loop in the config's unit: voltage
   * through {@code voltage}, or torque current through {@code current}.
   *
   * @param config the config whose output unit picks the request
   * @param voltage the mechanism's voltage control
   * @param current the mechanism's current control
   * @param <U> the output unit
   * @return the target
   * @throws IllegalArgumentException if the config's unit isn't volts or amps
   */
  default <U extends Unit> CharacterizationRoutine.Target<U> characterizationTarget(
      CharacterizationConfig<U> config, VoltageControl voltage, CurrentControl current) {
    return characterizationTarget(openLoop(config.getOutputUnit(), voltage, current));
  }

  /**
   * Returns a command that characterizes this mechanism on its own and prints the results.
   *
   * @param config the ramp, step, timeouts, settle time and legs
   * @param request how an output becomes a request to the motors, such as {@link
   *     VoltageControl#request(Measure)}
   * @param <U> the output unit
   * @return the command
   */
  default <U extends Unit> Command characterize(
      CharacterizationConfig<U> config, Function<Measure<U>, MotorRequest> request) {
    return new CharacterizationRoutine<>(
            config, core().getSubsystem(), List.of(characterizationTarget(request)))
        .command();
  }

  /**
   * Returns a command that characterizes this mechanism on its own, open loop in the config's unit,
   * and prints the results.
   *
   * @param config the ramp, step, timeouts, settle time and legs; volts or amps
   * @param voltage the mechanism's voltage control
   * @param current the mechanism's current control
   * @param <U> the output unit
   * @return the command
   * @throws IllegalArgumentException if the config's unit isn't volts or amps
   */
  default <U extends Unit> Command characterize(
      CharacterizationConfig<U> config, VoltageControl voltage, CurrentControl current) {
    return characterize(config, openLoop(config.getOutputUnit(), voltage, current));
  }

  /**
   * Returns how an output in a unit becomes an open-loop request: voltage through {@code voltage},
   * torque current through {@code current}.
   *
   * @param unit the output unit
   * @param voltage the voltage control
   * @param current the current control
   * @param <U> the output unit
   * @return the request function
   * @throws IllegalArgumentException if the unit isn't volts or amps
   */
  static <U extends Unit> Function<Measure<U>, MotorRequest> openLoop(
      U unit, VoltageControl voltage, CurrentControl current) {
    OutputType type =
        OutputType.of(unit)
            .filter(t -> t != OutputType.DUTY_CYCLE)
            .orElseThrow(
                () -> new IllegalArgumentException("Characterize in volts or amps, not " + unit));
    return type == OutputType.VOLTAGE
        ? output -> voltage.request(Volts.of(output.baseUnitMagnitude()))
        : output -> current.request(Amps.of(output.baseUnitMagnitude()));
  }
}
