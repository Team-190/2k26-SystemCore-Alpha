package edu.wpi.team190.gompeilib.sysid;

import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismCore;
import edu.wpi.team190.gompeilib.sysid.SysIdConfig.Leg;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.sysid.SysIdRoutineLog;
import org.wpilib.system.Timer;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.measure.Time;

/**
 * The SysId characterization routine: drives one or more mechanisms open loop through the legs in a
 * {@link SysIdConfig}, stopping each leg at a travel limit, on timeout, or when cancelled.
 *
 * <p>The routine idles every mechanism's core, then sends its requests straight to the motors. It
 * runs quasistatic forward, quasistatic reverse, dynamic forward and dynamic reverse (those the
 * config includes), waiting the settle time at zero output between them. A leg ends as soon as any
 * constrained mechanism reaches the limit in the leg's direction; a leg that would start there is
 * skipped. The output goes to zero whenever a leg ends.
 *
 * <p>Each mechanism logs the SysId state under {@code <topic>/SysId/State} (the state names
 * AdvantageScope's SysId analysis expects) and why the last leg ended under {@code
 * <topic>/SysId/EndReason}.
 *
 * @param <U> the output unit
 */
public final class CustomSysIdRoutine<U extends Unit> {
  /** Why a leg ended. */
  public enum EndReason {
    /** A constrained mechanism reached the limit in the leg's direction. */
    LIMIT,
    /** The leg ran for its full timeout. */
    TIMEOUT,
    /** A constrained mechanism was already at the limit, so the leg didn't run. */
    SKIPPED,
    /** The leg was interrupted. */
    CANCELLED
  }

  /**
   * A mechanism the routine drives.
   *
   * @param core the mechanism's core, which the routine idles and sends requests through
   * @param request how an output becomes a request to the mechanism's motors
   * @param constraint the travel the mechanism must stay inside, or empty for a continuous one
   * @param <U> the output unit
   */
  public record Target<U extends Unit>(
      MechanismCore core,
      Function<Measure<U>, MotorRequest> request,
      Optional<SysIdConstraint<?>> constraint) {
    public Target {
      Objects.requireNonNull(core, "core");
      Objects.requireNonNull(request, "request");
      Objects.requireNonNull(constraint, "constraint");
    }

    private void drive(Measure<U> output) {
      core.setControl(request.apply(output));
    }

    private boolean atLimit(Leg leg) {
      return constraint.isPresent() && constraint.get().atLimit(leg.isForward());
    }
  }

  private final SysIdConfig<U> config;
  private final Subsystem subsystem;
  private final List<Target<U>> targets;

  /**
   * Creates a routine.
   *
   * @param config the ramp, step, timeouts, settle time and legs
   * @param subsystem the subsystem that owns every mechanism; each leg requires it
   * @param targets the mechanisms to drive together
   * @throws IllegalArgumentException if there are no targets
   */
  public CustomSysIdRoutine(SysIdConfig<U> config, Subsystem subsystem, List<Target<U>> targets) {
    this.config = Objects.requireNonNull(config, "config");
    this.subsystem = Objects.requireNonNull(subsystem, "subsystem");
    this.targets = List.copyOf(targets);
    if (this.targets.isEmpty()) {
      throw new IllegalArgumentException("A SysId routine needs at least one mechanism");
    }
  }

  /**
   * Returns the full routine: idle every mechanism, then run each configured leg in order with the
   * settle time between them.
   *
   * @return the command
   */
  public Command command() {
    List<Command> commands = new ArrayList<>();
    commands.add(Commands.runOnce(() -> targets.forEach(target -> target.core().idle())));
    for (Leg leg : Leg.values()) {
      if (!config.getLegs().contains(leg)) {
        continue;
      }
      if (commands.size() > 1) {
        commands.add(Commands.defer(() -> Commands.waitTime(config.settleTime().get()), Set.of()));
      }
      commands.add(leg(leg));
    }
    return Commands.sequence(commands.toArray(Command[]::new)).withName("SysId");
  }

  /**
   * Returns a single leg. It doesn't idle the mechanisms first; {@link #command()} does.
   *
   * @param leg the leg
   * @return the command
   */
  public Command leg(Leg leg) {
    return new LegCommand(leg);
  }

  // Unit#ofBaseUnits only returns Measure<?> since U can't be reified here.
  @SuppressWarnings("unchecked")
  private Measure<U> output(double baseUnits) {
    return (Measure<U>) config.getOutputUnit().ofBaseUnits(baseUnits);
  }

  private static SysIdRoutineLog.State state(Leg leg) {
    return switch (leg) {
      case QUASISTATIC_FORWARD -> SysIdRoutineLog.State.QUASISTATIC_FORWARD;
      case QUASISTATIC_REVERSE -> SysIdRoutineLog.State.QUASISTATIC_REVERSE;
      case DYNAMIC_FORWARD -> SysIdRoutineLog.State.DYNAMIC_FORWARD;
      case DYNAMIC_REVERSE -> SysIdRoutineLog.State.DYNAMIC_REVERSE;
    };
  }

  private final class LegCommand extends Command {
    private final Leg leg;
    private final SysIdRoutineLog.State state;
    private final double sign;
    private final Timer timer = new Timer();

    private double rampRate;
    private double stepOutput;
    private Time timeout;
    private EndReason reason;

    private LegCommand(Leg leg) {
      this.leg = leg;
      state = state(leg);
      sign = leg.isForward() ? 1.0 : -1.0;
      addRequirements(subsystem);
      setName("sysid-" + state + "-" + subsystem.getName());
    }

    private boolean anyAtLimit() {
      return targets.stream().anyMatch(target -> target.atLimit(leg));
    }

    private void log(SysIdRoutineLog.State state) {
      for (Target<U> target : targets) {
        Logger.recordOutput(target.core().getTopic() + "/SysId/State", state.toString());
      }
    }

    @Override
    public void initialize() {
      reason = null;
      if (anyAtLimit()) {
        reason = EndReason.SKIPPED;
        return;
      }
      rampRate = config.rampRate().get().baseUnitMagnitude();
      stepOutput = config.stepOutput().get().baseUnitMagnitude();
      timeout = config.timeout(leg);
      timer.restart();
    }

    @Override
    public void execute() {
      if (reason != null) {
        return;
      }
      Measure<U> output =
          output(sign * (leg.isQuasistatic() ? timer.get() * rampRate : stepOutput));
      targets.forEach(target -> target.drive(output));
      log(state);
    }

    @Override
    public boolean isFinished() {
      if (reason != null) {
        return true;
      }
      if (anyAtLimit()) {
        reason = EndReason.LIMIT;
      } else if (timer.hasElapsed(timeout)) {
        reason = EndReason.TIMEOUT;
      }
      return reason != null;
    }

    @Override
    public void end(boolean interrupted) {
      if (interrupted) {
        reason = EndReason.CANCELLED;
      }
      Measure<U> zero = output(0);
      targets.forEach(target -> target.drive(zero));
      timer.stop();
      log(SysIdRoutineLog.State.NONE);
      for (Target<U> target : targets) {
        Logger.recordOutput(target.core().getTopic() + "/SysId/EndReason", reason.toString());
      }
    }
  }
}
