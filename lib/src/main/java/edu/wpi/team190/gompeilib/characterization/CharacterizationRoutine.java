package edu.wpi.team190.gompeilib.characterization;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig.Leg;
import edu.wpi.team190.gompeilib.characterization.FeedforwardFit.Result;
import edu.wpi.team190.gompeilib.characterization.FeedforwardFit.Sample;
import edu.wpi.team190.gompeilib.io.motor.config.GravityType;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismCore;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
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
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

/**
 * Characterizes one or more mechanisms driven together, in whatever unit they're controlled in
 * (volts or torque current). Each leg ramps (quasistatic) or steps (dynamic) an open-loop output
 * while the routine samples every mechanism's effort, position, velocity and acceleration. Once the
 * legs finish, it fits {@code u = kS·sign(ω) + kV·ω + kA·α [+ kG·g(θ)]} to each mechanism's samples
 * and prints a report with the fit's quality, any warnings, and the {@link
 * edu.wpi.team190.gompeilib.io.physics.PlantCharacterization} and gains to paste into the
 * mechanism's constants. The values are also logged under {@code <topic>/Characterization}.
 *
 * <p>A leg ends when it times out, or, for a mechanism with a {@link CharacterizationConstraint},
 * when any mechanism nears the travel limit in its direction. A leg that starts inside that margin
 * is skipped. The routine logs each leg's state in WPILib SysId's format, so a log can still be
 * checked in the SysId tool.
 *
 * @param <U> the output unit
 */
public final class CharacterizationRoutine<U extends Unit> {
  /** Why a leg ended. */
  public enum EndReason {
    /** A mechanism reached the travel limit in the leg's direction. */
    LIMIT,
    /** The leg ran for its full timeout. */
    TIMEOUT,
    /** A mechanism was already at the limit when the leg started. */
    SKIPPED,
    /** The leg was interrupted. */
    CANCELLED
  }

  /**
   * Something the routine drives and samples.
   *
   * @param <U> the output unit
   */
  public interface Target<U extends Unit> {
    /**
     * Returns where the target logs, which also names it in the report.
     *
     * @return the log topic
     */
    String topic();

    /** Stops whatever the target is doing before the routine starts. */
    void idle();

    /**
     * Applies an open-loop output.
     *
     * @param output the output
     */
    void drive(Measure<U> output);

    /**
     * Returns the target's state now.
     *
     * @param effort whether to report voltage or torque current as the effort
     * @return the sample
     */
    Sample sample(OutputType effort);

    /**
     * Returns whether the target is at the travel limit in a leg's direction.
     *
     * @param leg the leg
     * @return true if the leg must stop
     */
    boolean atLimit(Leg leg);

    /**
     * Returns how gravity loads the target.
     *
     * @return the gravity type, or empty if gravity doesn't load it
     */
    Optional<GravityType> gravity();

    /**
     * Returns the drum that turns the target's rotation into travel, so the report prints gains per
     * meter.
     *
     * @return the drum's radius, or empty for gains per rotation
     */
    Optional<Distance> drumRadius();
  }

  /**
   * A mechanism as a target, sampled from its leader motor.
   *
   * @param core the mechanism
   * @param request how an output becomes a request to the motors
   * @param constraint the travel the mechanism must stay inside, or empty for a continuous one
   * @param gravity how gravity loads the mechanism, or empty if it doesn't
   * @param drumRadius the mechanism's drum, or empty for an angular mechanism
   * @param <U> the output unit
   */
  public record MechanismTarget<U extends Unit>(
      MechanismCore core,
      Function<Measure<U>, MotorRequest> request,
      Optional<CharacterizationConstraint<?>> constraint,
      Optional<GravityType> gravity,
      Optional<Distance> drumRadius)
      implements Target<U> {
    public MechanismTarget {
      Objects.requireNonNull(core, "core");
      Objects.requireNonNull(request, "request");
      Objects.requireNonNull(constraint, "constraint");
      Objects.requireNonNull(gravity, "gravity");
      Objects.requireNonNull(drumRadius, "drumRadius");
    }

    @Override
    public String topic() {
      return core.getTopic();
    }

    @Override
    public void idle() {
      core.idle();
    }

    @Override
    public void drive(Measure<U> output) {
      core.setControl(request.apply(output));
    }

    @Override
    public Sample sample(OutputType effort) {
      return CharacterizationRoutine.sample(core.getMotors().leader(), effort);
    }

    @Override
    public boolean atLimit(Leg leg) {
      return constraint.isPresent() && constraint.get().atLimit(leg.isForward());
    }
  }

  private final CharacterizationConfig<U> config;
  private final Subsystem subsystem;
  private final List<Target<U>> targets;
  private final OutputType effort;
  private final List<List<Sample>> samples = new ArrayList<>();

  /**
   * Creates a routine.
   *
   * @param config the ramp, step, timeouts, settle time and legs
   * @param subsystem the subsystem that owns every target; each leg requires it
   * @param targets the targets to drive together
   * @throws IllegalArgumentException if there are no targets, or the output isn't volts or amps
   */
  public CharacterizationRoutine(
      CharacterizationConfig<U> config, Subsystem subsystem, List<Target<U>> targets) {
    this.config = Objects.requireNonNull(config, "config");
    this.subsystem = Objects.requireNonNull(subsystem, "subsystem");
    this.targets = List.copyOf(targets);
    if (this.targets.isEmpty()) {
      throw new IllegalArgumentException("A characterization needs at least one target");
    }
    effort =
        OutputType.of(config.getOutputUnit())
            .filter(type -> type != OutputType.DUTY_CYCLE)
            .orElseThrow(
                () ->
                    new IllegalArgumentException(
                        "Characterize in volts or amps, not " + config.getOutputUnit()));
    for (int i = 0; i < this.targets.size(); i++) {
      samples.add(new ArrayList<>());
    }
  }

  /**
   * Reads a motor's telemetry as a sample at its gearbox output.
   *
   * @param telemetry the motor's telemetry
   * @param effort whether to read voltage or torque current as the effort
   * @return the sample
   */
  public static Sample sample(MotorTelemetry telemetry, OutputType effort) {
    double motorKV = telemetry.motorKV.baseUnitMagnitude();
    return new Sample(
        effort == OutputType.CURRENT
            ? telemetry.torqueCurrent.in(Amps)
            : telemetry.appliedVoltage.in(Volts),
        telemetry.position.in(Radians),
        telemetry.velocity.in(RadiansPerSecond),
        telemetry.acceleration.in(RadiansPerSecondPerSecond),
        motorKV > 0 ? telemetry.rotorVelocity.in(RadiansPerSecond) / motorKV : Double.NaN);
  }

  /**
   * Returns the full routine: idle every target and forget earlier samples, run each configured leg
   * in order with the settle time between them, then fit every target and print the report.
   *
   * @return the command
   */
  public Command command() {
    List<Command> commands = new ArrayList<>();
    commands.add(
        Commands.runOnce(
            () -> {
              samples.forEach(List::clear);
              targets.forEach(Target::idle);
            }));
    boolean first = true;
    for (Leg leg : Leg.values()) {
      if (!config.getLegs().contains(leg)) {
        continue;
      }
      if (!first) {
        commands.add(Commands.defer(() -> Commands.waitTime(config.settleTime().get()), Set.of()));
      }
      first = false;
      commands.add(leg(leg));
    }
    commands.add(Commands.defer(() -> Commands.print(report()), Set.of()));
    return Commands.sequence(commands.toArray(Command[]::new)).withName("Characterization");
  }

  /**
   * Returns a single leg. It doesn't idle the targets or forget earlier samples; {@link #command()}
   * does.
   *
   * @param leg the leg
   * @return the command
   */
  public Command leg(Leg leg) {
    return new LegCommand(leg);
  }

  /**
   * Returns the samples taken from a target so far.
   *
   * @param target the target's index
   * @return the samples
   */
  public List<Sample> samples(int target) {
    return List.copyOf(samples.get(target));
  }

  /**
   * Fits a target's samples so far.
   *
   * @param target the target's index
   * @return the fit
   * @throws IllegalArgumentException if the samples can't support a fit
   */
  public Result fit(int target) {
    return FeedforwardFit.fit(samples.get(target), targets.get(target).gravity());
  }

  /**
   * Fits every target's samples, logs the results, and returns the report.
   *
   * @return the report
   */
  public String report() {
    StringBuilder report = new StringBuilder();
    for (int i = 0; i < targets.size(); i++) {
      Target<U> target = targets.get(i);
      report.append("********** Characterization: ").append(target.topic()).append(" **********\n");
      try {
        Result result = fit(i);
        log(target.topic(), result);
        report.append(format(result, effort, target.drumRadius()));
      } catch (IllegalArgumentException e) {
        report.append("FAILED: ").append(e.getMessage()).append('\n');
      }
    }
    return report.toString();
  }

  private static void log(String topic, Result result) {
    String prefix = topic + "/Characterization/";
    Logger.recordOutput(prefix + "kS", result.kS().value());
    Logger.recordOutput(prefix + "kV", result.kV().value());
    Logger.recordOutput(prefix + "kA", result.kA().value());
    Logger.recordOutput(prefix + "kG", result.kG().value());
    Logger.recordOutput(prefix + "RSquared", result.rSquared());
    Logger.recordOutput(prefix + "RMSE", result.rmse());
    Logger.recordOutput(prefix + "Samples", result.samples());
  }

  /**
   * Formats a fit as the report's body: its quality, warnings, and paste-ready snippets.
   *
   * @param result the fit
   * @param effort the effort's type
   * @param drumRadius the drum, to print the gains per meter, or empty to print them per rotation
   * @return the text
   */
  static String format(Result result, OutputType effort, Optional<Distance> drumRadius) {
    boolean voltage = effort == OutputType.VOLTAGE;
    String unit = voltage ? "Volts" : "Amps";
    String symbol = voltage ? "V" : "A";
    StringBuilder text = new StringBuilder();
    text.append(
        String.format(
            Locale.ROOT,
            "Samples: %d | R²: %.4f | RMSE: %.4g %s%n",
            result.samples(),
            result.rSquared(),
            result.rmse(),
            symbol));
    for (String warning : result.warnings(voltage)) {
      text.append("WARNING: ").append(warning).append('\n');
    }

    double rotation = 2 * Math.PI;
    boolean gravity = result.gravity().isPresent();
    text.append("Plant:\n")
        .append("PlantCharacterization.")
        .append(voltage ? "voltage()" : "current()")
        .append('\n');
    appendTerms(
        text,
        unit,
        "RotationsPerSecond",
        "RotationsPerSecondPerSecond",
        result,
        result.kV().value() * rotation,
        result.kA().value() * rotation,
        gravity);
    if (gravity) {
      text.append("    .withGravityType(GravityType.").append(result.gravity().get()).append(")\n");
    }
    text.append("    .build()\n");

    text.append("Gains:\n");
    if (drumRadius.isPresent()) {
      double meters = drumRadius.get().in(Meters);
      appendTerms(
          text,
          unit,
          "MetersPerSecond",
          "MetersPerSecondPerSecond",
          result,
          result.kV().value() / meters,
          result.kA().value() / meters,
          gravity);
    } else {
      appendTerms(
          text,
          unit,
          "RotationsPerSecond",
          "RotationsPerSecondPerSecond",
          result,
          result.kV().value() * rotation,
          result.kA().value() * rotation,
          gravity);
    }
    return text.toString();
  }

  private static void appendTerms(
      StringBuilder text,
      String unit,
      String velocityUnit,
      String accelerationUnit,
      Result result,
      double kV,
      double kA,
      boolean gravity) {
    text.append(term("KS", unit + ".of(%s)", result.kS().value()));
    text.append(term("KV", unit + ".per(" + velocityUnit + ").ofNative(%s)", kV));
    text.append(term("KA", unit + ".per(" + accelerationUnit + ").ofNative(%s)", kA));
    if (gravity) {
      text.append(term("KG", unit + ".of(%s)", result.kG().value()));
    }
  }

  private static String term(String name, String format, double value) {
    return "    .with"
        + name
        + "("
        + String.format(Locale.ROOT, format, String.format(Locale.ROOT, "%.6g", value))
        + ")\n";
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
      setName("characterize-" + state + "-" + subsystem.getName());
    }

    private boolean anyAtLimit() {
      return targets.stream().anyMatch(target -> target.atLimit(leg));
    }

    private void log(SysIdRoutineLog.State state) {
      for (Target<U> target : targets) {
        Logger.recordOutput(target.topic() + "/Characterization/State", state.toString());
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
      for (int i = 0; i < targets.size(); i++) {
        samples.get(i).add(targets.get(i).sample(effort));
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
        Logger.recordOutput(target.topic() + "/Characterization/EndReason", reason.toString());
      }
    }
  }
}
