package edu.wpi.team190.gompeilib.robot.common.characterization;

import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConstraint;
import edu.wpi.team190.gompeilib.characterization.CharacterizationRoutine;
import edu.wpi.team190.gompeilib.io.motor.config.GravityType;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.robot.common.core.Mechanism;
import edu.wpi.team190.gompeilib.robot.common.openloop.OpenLoopControl;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import org.wpilib.command2.Command;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Distance;

/**
 * A mechanism that can be characterized with the shared {@link CharacterizationRoutine}. A
 * positional mechanism overrides {@link #characterizationConstraint()} with its travel limits; a
 * continuous one keeps the default and runs every leg to its timeout. A mechanism gravity loads
 * overrides {@link #characterizationGravity()} so the fit includes {@code kG}, and one that travels
 * linearly overrides {@link #characterizationDistancePerRadian()} so the report prints gains per
 * meter.
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
   * Returns how far the mechanism travels per radian of its gearbox output, for a linear mechanism.
   *
   * @return the distance per radian, or empty for an angular mechanism
   */
  default Optional<Distance> characterizationDistancePerRadian() {
    return Optional.empty();
  }

  /**
   * Returns this mechanism as a routine target, for characterizing several mechanisms together.
   *
   * @param request how an output becomes a request to the motors, such as {@link
   *     OpenLoopControl#request(Measure)}
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
        characterizationDistancePerRadian());
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
      CharacterizationConfig<U> config,
      OpenLoopControl<VoltageUnit> voltage,
      OpenLoopControl<CurrentUnit> current) {
    return characterizationTarget(OpenLoopControl.select(config.getOutputUnit(), voltage, current));
  }

  /**
   * Returns a command that characterizes this mechanism on its own and prints the results.
   *
   * @param config the ramp, step, timeouts, settle time and legs
   * @param request how an output becomes a request to the motors, such as {@link
   *     OpenLoopControl#request(Measure)}
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
      CharacterizationConfig<U> config,
      OpenLoopControl<VoltageUnit> voltage,
      OpenLoopControl<CurrentUnit> current) {
    return characterize(config, OpenLoopControl.select(config.getOutputUnit(), voltage, current));
  }
}
