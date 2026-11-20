package edu.wpi.team190.gompeilib.robot.common.velocity;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLib;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.outputs.Feedforward;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.motor.outputs.VelocityProfileMode;
import edu.wpi.team190.gompeilib.io.motor.outputs.VelocityRequest;
import edu.wpi.team190.gompeilib.robot.common.core.GainSlots;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismConversion;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismCore;
import java.util.Objects;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;

/**
 * Closed-loop velocity control, in the mechanism's velocity unit. The feedforward's kind picks the
 * output the loop drives: voltage, torque current, or duty cycle.
 *
 * @param <V> the mechanism's velocity unit
 */
public final class VelocityControl<V extends Unit> implements MechanismCore.Control {
  private final MechanismCore core;
  private final MechanismConversion<?, V, ?> conversion;
  private final Supplier<? extends Measure<V>> tolerance;
  private final boolean foc;
  private final GainSlots gainSlots = new GainSlots();

  @Getter private Setpoint<V> goal;
  @Getter private Feedforward feedforward;
  @Getter private VelocityProfileMode profileMode = VelocityProfileMode.UNPROFILED;
  @Getter private RequestOptions options = RequestOptions.DEFAULT;

  /**
   * Creates a velocity control on the voltage loop and registers it with the core.
   *
   * @param core the mechanism's core
   * @param conversion the mechanism's conversion
   * @param goal the initial goal
   * @param tolerance how close the velocity must be to count as at the goal, read each check
   * @param foc whether requests use field-oriented control
   */
  public VelocityControl(
      MechanismCore core,
      MechanismConversion<?, V, ?> conversion,
      Setpoint<V> goal,
      Supplier<? extends Measure<V>> tolerance,
      boolean foc) {
    this(core, conversion, goal, tolerance, foc, OutputType.VOLTAGE);
  }

  /**
   * Creates a velocity control with zero feedforward on an output type and registers it with the
   * core.
   *
   * @param core the mechanism's core
   * @param conversion the mechanism's conversion
   * @param goal the initial goal
   * @param tolerance how close the velocity must be to count as at the goal, read each check
   * @param foc whether requests use field-oriented control
   * @param outputType the output the loop drives
   */
  public VelocityControl(
      MechanismCore core,
      MechanismConversion<?, V, ?> conversion,
      Setpoint<V> goal,
      Supplier<? extends Measure<V>> tolerance,
      boolean foc,
      OutputType outputType) {
    this.core = core;
    this.conversion = Objects.requireNonNull(conversion, "conversion");
    this.goal = Objects.requireNonNull(goal, "goal");
    this.tolerance = Objects.requireNonNull(tolerance, "tolerance");
    this.foc = foc;
    this.feedforward = Feedforward.zero(outputType);
    core.add(this);
  }

  /**
   * Sets the goal, keeping the feedforward and output type, and makes this the active control.
   *
   * @param velocity the goal
   */
  public void setGoal(Measure<V> velocity) {
    goal.setSetpoint(velocity);
    core.activate(this);
  }

  /**
   * Replaces the goal, keeping the feedforward and output type, and makes this the active control.
   *
   * @param goal the goal
   */
  public void setGoal(Setpoint<V> goal) {
    this.goal = Objects.requireNonNull(goal, "goal");
    core.activate(this);
  }

  /**
   * Sets the goal and the feedforward, whose kind picks the output type, and makes this active.
   *
   * @param velocity the goal
   * @param feedforward the feedforward
   */
  public void setGoal(Measure<V> velocity, Feedforward feedforward) {
    this.feedforward = Objects.requireNonNull(feedforward, "feedforward");
    setGoal(velocity);
  }

  /**
   * Replaces the goal and sets the feedforward, whose kind picks the output type, and makes this
   * active.
   *
   * @param goal the goal
   * @param feedforward the feedforward
   */
  public void setGoal(Setpoint<V> goal, Feedforward feedforward) {
    Objects.requireNonNull(goal, "goal");
    this.feedforward = Objects.requireNonNull(feedforward, "feedforward");
    setGoal(goal);
  }

  /**
   * Returns the output the loop drives, which the feedforward's kind picks.
   *
   * @return the output type
   */
  public OutputType getOutputType() {
    return feedforward.type();
  }

  /**
   * Switches the loop to an output type, with zero feedforward.
   *
   * @param outputType the output type
   */
  public void setOutputType(OutputType outputType) {
    feedforward = Feedforward.zero(outputType);
  }

  /**
   * Returns the gain slot the active output type uses.
   *
   * @return the slot
   */
  public GainSlot getGainSlot() {
    return gainSlots.get(getOutputType());
  }

  /**
   * Uses one gain slot for every output type, from the next loop on.
   *
   * @param slot the slot
   */
  public void setGainSlot(GainSlot slot) {
    gainSlots.set(slot);
  }

  /**
   * Uses a gain slot for one output type, from the next loop on.
   *
   * @param outputType the output type
   * @param slot the slot
   */
  public void setGainSlot(OutputType outputType, GainSlot slot) {
    gainSlots.set(outputType, slot);
  }

  /**
   * Sets how the loop reaches the goal.
   *
   * @param profileMode the mode
   */
  public void setProfileMode(VelocityProfileMode profileMode) {
    this.profileMode = Objects.requireNonNull(profileMode, "profileMode");
  }

  public void setOptions(RequestOptions options) {
    this.options = Objects.requireNonNull(options, "options");
  }

  /**
   * Returns the leader's velocity, in the mechanism's unit.
   *
   * @return the velocity
   */
  public Measure<V> getVelocity() {
    return conversion.fromMotorVelocity(core.getMotors().leader().velocity);
  }

  public boolean atGoal() {
    return atGoal(goal.getNewSetpoint());
  }

  public boolean atGoal(Measure<V> reference) {
    return Math.abs(getVelocity().baseUnitMagnitude() - reference.baseUnitMagnitude())
        <= tolerance.get().baseUnitMagnitude();
  }

  /**
   * Waits one loop, so periodic applies the latest goal, then until the velocity is at the goal.
   *
   * @return the command
   */
  public Command waitUntilAtGoal() {
    return Commands.waitSeconds(GompeiLib.getLoopPeriod())
        .andThen(Commands.waitUntil(this::atGoal));
  }

  @Override
  public String name() {
    return switch (getOutputType()) {
      case CURRENT -> "VelocityCurrent";
      case DUTY_CYCLE -> "VelocityDutyCycle";
      case VOLTAGE -> "VelocityVoltage";
    };
  }

  @Override
  public MotorRequest request() {
    return VelocityRequest.of(
        profileMode,
        conversion.toMotorVelocity(goal.getNewSetpoint()),
        RadiansPerSecondPerSecond.of(0.0),
        feedforward,
        foc,
        getGainSlot(),
        options);
  }

  @Override
  public void periodic(String topic) {
    Logger.recordOutputMeasure(topic + "/Velocity/Goal", goal.getSetpoint());
    Logger.recordOutputMeasure(topic + "/Velocity/Offset", goal.getOffset());
    Logger.recordOutputMeasure(topic + "/Velocity/Measured", getVelocity());
    switch (feedforward) {
      case Feedforward.Volts(var voltage) ->
          Logger.recordOutputMeasure(topic + "/Velocity/Feedforward", voltage);
      case Feedforward.Amps(var current) ->
          Logger.recordOutputMeasure(topic + "/Velocity/Feedforward", current);
      case Feedforward.DutyCycle(var dutyCycle) ->
          Logger.recordOutput(topic + "/Velocity/Feedforward", dutyCycle);
    }
    Logger.recordOutput(topic + "/Velocity/AtGoal", atGoal());
    Logger.recordOutput(topic + "/Velocity/OutputType", getOutputType().name());
  }
}
