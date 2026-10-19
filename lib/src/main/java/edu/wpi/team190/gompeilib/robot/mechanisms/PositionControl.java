package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLib;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.motor.outputs.PositionRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import java.util.Objects;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

/**
 * Closed-loop position control, in the mechanism's position unit. Can drive voltage,
 * torque-producing current, or duty-cycle outputs, with optional feedforward and profile modes.
 *
 * @param <P> the mechanism's position unit
 */
public final class PositionControl<P extends Unit> implements MechanismCore.Control {
  /** The motion profile mode for position control. */
  public enum ProfileMode {
    /** Trapezoidal motion profiling (Motion Magic). */
    PROFILED,
    /** Unprofiled closed-loop position control. */
    UNPROFILED,
    /** Exponential motion profiling (Motion Magic Expo). */
    EXPONENTIAL
  }

  private final MechanismCore core;
  private final MechanismConversion<P, ?, ?> conversion;
  private final Supplier<? extends Measure<P>> tolerance;
  private final boolean foc;

  @Getter private Setpoint<P> goal;
  @Getter @Setter private GainSlot gainSlot = GainSlot.ZERO;
  @Getter @Setter private OutputType outputType;
  @Getter @Setter private ProfileMode profileMode = ProfileMode.PROFILED;

  @Getter private Voltage voltageFeedforward = Volts.of(0.0);
  @Getter private Current currentFeedforward = Amps.of(0.0);
  @Getter private double dutyCycleFeedforward = 0.0;

  /**
   * Creates a position control and registers it with the core, defaulting to voltage output.
   *
   * @param core the mechanism's core
   * @param conversion the mechanism's conversion
   * @param goal the initial goal
   * @param tolerance how close the position must be to count as at the goal, read each check
   * @param foc whether requests use field-oriented control
   */
  public PositionControl(
      MechanismCore core,
      MechanismConversion<P, ?, ?> conversion,
      Setpoint<P> goal,
      Supplier<? extends Measure<P>> tolerance,
      boolean foc) {
    this(core, conversion, goal, tolerance, foc, OutputType.VOLTAGE);
  }

  /**
   * Creates a position control with an explicit output type and registers it with the core.
   *
   * @param core the mechanism's core
   * @param conversion the mechanism's conversion
   * @param goal the initial goal
   * @param tolerance how close the position must be to count as at the goal, read each check
   * @param foc whether requests use field-oriented control
   * @param outputType the output type to use
   */
  public PositionControl(
      MechanismCore core,
      MechanismConversion<P, ?, ?> conversion,
      Setpoint<P> goal,
      Supplier<? extends Measure<P>> tolerance,
      boolean foc,
      OutputType outputType) {
    this.core = core;
    this.conversion = Objects.requireNonNull(conversion, "conversion");
    this.goal = Objects.requireNonNull(goal, "goal");
    this.tolerance = Objects.requireNonNull(tolerance, "tolerance");
    this.foc = foc;
    this.outputType = Objects.requireNonNull(outputType, "outputType");
    core.add(this);
  }

  /**
   * Sets the goal, preserving the active output type, and makes this the active control.
   *
   * @param position the goal
   */
  public void setGoal(Measure<P> position) {
    goal.setSetpoint(position);
    core.activate(this);
  }

  /**
   * Replaces the goal, preserving the active output type, and makes this the active control.
   *
   * @param goal the goal
   */
  public void setGoal(Setpoint<P> goal) {
    this.goal = Objects.requireNonNull(goal, "goal");
    core.activate(this);
  }

  /**
   * Sets the goal with a voltage feedforward, runs the voltage loop, and makes this active.
   *
   * @param position the goal
   * @param feedforward the voltage feedforward
   */
  public void setGoal(Measure<P> position, Voltage feedforward) {
    goal.setSetpoint(position);
    this.voltageFeedforward = Objects.requireNonNull(feedforward, "feedforward");
    this.outputType = OutputType.VOLTAGE;
    core.activate(this);
  }

  /**
   * Replaces the goal with a voltage feedforward, runs the voltage loop, and makes this active.
   *
   * @param goal the goal
   * @param feedforward the voltage feedforward
   */
  public void setGoal(Setpoint<P> goal, Voltage feedforward) {
    this.goal = Objects.requireNonNull(goal, "goal");
    this.voltageFeedforward = Objects.requireNonNull(feedforward, "feedforward");
    this.outputType = OutputType.VOLTAGE;
    core.activate(this);
  }

  /**
   * Sets the goal with a current feedforward, runs the current loop, and makes this active.
   *
   * @param position the goal
   * @param feedforward the torque-current feedforward
   */
  public void setGoal(Measure<P> position, Current feedforward) {
    goal.setSetpoint(position);
    this.currentFeedforward = Objects.requireNonNull(feedforward, "feedforward");
    this.outputType = OutputType.CURRENT;
    core.activate(this);
  }

  /**
   * Replaces the goal with a current feedforward, runs the current loop, and makes this active.
   *
   * @param goal the goal
   * @param feedforward the torque-current feedforward
   */
  public void setGoal(Setpoint<P> goal, Current feedforward) {
    this.goal = Objects.requireNonNull(goal, "goal");
    this.currentFeedforward = Objects.requireNonNull(feedforward, "feedforward");
    this.outputType = OutputType.CURRENT;
    core.activate(this);
  }

  /**
   * Sets the goal with a duty-cycle feedforward, runs the duty-cycle loop, and makes this active.
   *
   * @param position the goal
   * @param feedforward the duty-cycle feedforward in [-1, 1]
   */
  public void setGoal(Measure<P> position, double feedforward) {
    goal.setSetpoint(position);
    this.dutyCycleFeedforward = feedforward;
    this.outputType = OutputType.DUTY_CYCLE;
    core.activate(this);
  }

  /**
   * Replaces the goal with a duty-cycle feedforward, runs the duty-cycle loop, and makes this
   * active.
   *
   * @param goal the goal
   * @param feedforward the duty-cycle feedforward in [-1, 1]
   */
  public void setGoal(Setpoint<P> goal, double feedforward) {
    this.goal = Objects.requireNonNull(goal, "goal");
    this.dutyCycleFeedforward = feedforward;
    this.outputType = OutputType.DUTY_CYCLE;
    core.activate(this);
  }

  /**
   * Returns the leader's position, in the mechanism's unit.
   *
   * @return the position
   */
  public Measure<P> getPosition() {
    return conversion.fromMotorPosition(core.getMotors().leader().position);
  }

  /**
   * Resets every motor's reported position.
   *
   * @param position the mechanism's position now
   */
  public void setPosition(Measure<P> position) {
    core.getMotors().setPosition(conversion.toMotorPosition(position));
  }

  public boolean atGoal() {
    return atGoal(goal.getNewSetpoint());
  }

  public boolean atGoal(Measure<P> reference) {
    return Math.abs(getPosition().baseUnitMagnitude() - reference.baseUnitMagnitude())
        <= tolerance.get().baseUnitMagnitude();
  }

  /**
   * Waits one loop, so periodic applies the latest goal, then until the position is at the goal.
   *
   * @return the command
   */
  public Command waitUntilAtGoal() {
    return Commands.waitSeconds(GompeiLib.getLoopPeriod())
        .andThen(Commands.waitUntil(this::atGoal));
  }

  @Override
  public String name() {
    return switch (outputType) {
      case CURRENT -> "PositionCurrent";
      case DUTY_CYCLE -> "PositionDutyCycle";
      case VOLTAGE -> "PositionVoltage";
    };
  }

  @Override
  public MotorRequest request() {
    Angle position = conversion.toMotorPosition(goal.getNewSetpoint());
    return switch (outputType) {
      case CURRENT ->
          switch (profileMode) {
            case PROFILED ->
                new PositionRequest.ProfiledPositionCurrent(
                    position, currentFeedforward, gainSlot, RequestOptions.DEFAULT);
            case UNPROFILED ->
                new PositionRequest.PositionCurrent(
                    position,
                    RadiansPerSecond.of(0.0),
                    currentFeedforward,
                    gainSlot,
                    RequestOptions.DEFAULT);
            case EXPONENTIAL ->
                new PositionRequest.ExponentialProfiledPositionCurrent(
                    position, currentFeedforward, gainSlot, RequestOptions.DEFAULT);
          };
      case DUTY_CYCLE ->
          switch (profileMode) {
            case PROFILED ->
                new PositionRequest.ProfiledPositionDutyCycle(
                    position, dutyCycleFeedforward, foc, gainSlot, RequestOptions.DEFAULT);
            case UNPROFILED ->
                new PositionRequest.PositionDutyCycle(
                    position,
                    RadiansPerSecond.of(0.0),
                    dutyCycleFeedforward,
                    foc,
                    gainSlot,
                    RequestOptions.DEFAULT);
            case EXPONENTIAL ->
                new PositionRequest.ExponentialProfiledPositionDutyCycle(
                    position, dutyCycleFeedforward, foc, gainSlot, RequestOptions.DEFAULT);
          };
      case VOLTAGE ->
          switch (profileMode) {
            case PROFILED ->
                new PositionRequest.ProfiledPositionVoltage(
                    position, voltageFeedforward, foc, gainSlot, RequestOptions.DEFAULT);
            case UNPROFILED ->
                new PositionRequest.PositionVoltage(
                    position,
                    RadiansPerSecond.of(0.0),
                    voltageFeedforward,
                    foc,
                    gainSlot,
                    RequestOptions.DEFAULT);
            case EXPONENTIAL ->
                new PositionRequest.ExponentialProfiledPositionVoltage(
                    position, voltageFeedforward, foc, gainSlot, RequestOptions.DEFAULT);
          };
    };
  }

  @Override
  public void periodic(String topic) {
    Logger.recordOutputMeasure(topic + "/Position/Goal", goal.getSetpoint());
    Logger.recordOutputMeasure(topic + "/Position/Offset", goal.getOffset());
    Logger.recordOutputMeasure(topic + "/Position/Measured", getPosition());
    Logger.recordOutput(topic + "/Position/AtGoal", atGoal());
    Logger.recordOutput(topic + "/Position/GainSlot", gainSlot.name());
    Logger.recordOutput(topic + "/Position/OutputType", outputType.name());
  }
}
