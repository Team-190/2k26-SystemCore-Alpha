package edu.wpi.team190.gompeilib.robot.mechanisms;

import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.sysid.CustomSysIdRoutine;
import edu.wpi.team190.gompeilib.sysid.SysIdConfig;
import edu.wpi.team190.gompeilib.sysid.SysIdConstraint;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import org.wpilib.command2.Command;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;

/**
 * A mechanism that can be characterized with the shared {@link CustomSysIdRoutine}. A positional
 * mechanism overrides {@link #sysIdConstraint()} with its travel limits; a continuous one keeps the
 * default and runs every leg to its timeout.
 */
public interface Characterizable extends Mechanism {
  /**
   * Returns the travel the routine must keep the mechanism inside.
   *
   * @return the constraint, or empty for a continuous mechanism
   */
  default Optional<SysIdConstraint<?>> sysIdConstraint() {
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
  default <U extends Unit> CustomSysIdRoutine.Target<U> sysIdTarget(
      Function<Measure<U>, MotorRequest> request) {
    return new CustomSysIdRoutine.Target<>(core(), request, sysIdConstraint());
  }

  /**
   * Returns a command that characterizes this mechanism on its own.
   *
   * @param config the ramp, step, timeouts, settle time and legs
   * @param request how an output becomes a request to the motors, such as {@link
   *     VoltageControl#request(Measure)}
   * @param <U> the output unit
   * @return the command
   */
  default <U extends Unit> Command characterize(
      SysIdConfig<U> config, Function<Measure<U>, MotorRequest> request) {
    return new CustomSysIdRoutine<>(config, core().getSubsystem(), List.of(sysIdTarget(request)))
        .command();
  }
}
