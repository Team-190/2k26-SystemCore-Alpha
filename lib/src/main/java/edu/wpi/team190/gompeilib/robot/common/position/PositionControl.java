package edu.wpi.team190.gompeilib.robot.common.position;

import edu.wpi.team190.gompeilib.GompeiLib;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.outputs.Feedforward;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.motor.outputs.PositionRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.ProfileMode;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
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
 * Closed-loop position control, in the mechanism's position unit. The feedforward's kind picks the
 * output the loop drives: voltage, torque current, or duty cycle.
 *
 * @param <P> the mechanism's position unit
 */
public final class PositionControl<P extends Unit> implements MechanismCore.Control {
  private final MechanismCore core;
  private final MechanismConversion<P, ?, ?> conversion;
  private final Supplier<? extends Measure<P>> tolerance;
  private final boolean foc;
  private final GainSlots gainSlots = new GainSlots();

  @Getter private Setpoint<P> goal;
  @Getter private Feedforward feedforward;
  @Getter private ProfileMode profileMode = ProfileMode.PROFILED;
  @Getter private RequestOptions options = RequestOptions.DEFAULT;

  /**
   * Creates a position control on the voltage loop and registers it with the core.
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
   * Creates a position control with zero feedforward on an output type and registers it with the
   * core.
   *
   * @param core the mechanism's core
   * @param conversion the mechanism's conversion
   * @param goal the initial goal
   * @param tolerance how close the position must be to count as at the goal, read each check
   * @param foc whether requests use field-oriented control
   * @param outputType the output the loop drives
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
    this.feedforward = Feedforward.zero(outputType);
    core.add(this);
  }

  /**
   * Sets the goal, keeping the feedforward and output type, and makes this the active control.
   *
   * @param position the goal
   */
  public void setGoal(Measure<P> position) {
    goal.setSetpoint(position);
    core.activate(this);
  }

  /**
   * Replaces the goal, keeping the feedforward and output type, and makes this the active control.
   *
   * @param goal the goal
   */
  public void setGoal(Setpoint<P> goal) {
    this.goal = Objects.requireNonNull(goal, "goal");
    core.activate(this);
  }

  /**
   * Sets the goal and the feedforward, whose kind picks the output type, and makes this active.
   *
   * @param position the goal
   * @param feedforward the feedforward
   */
  public void setGoal(Measure<P> position, Feedforward feedforward) {
    this.feedforward = Objects.requireNonNull(feedforward, "feedforward");
    setGoal(position);
  }

  /**
   * Replaces the goal and sets the feedforward, whose kind picks the output type, and makes this
   * active.
   *
   * @param goal the goal
   * @param feedforward the feedforward
   */
  public void setGoal(Setpoint<P> goal, Feedforward feedforward) {
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

  public void setProfileMode(ProfileMode profileMode) {
    this.profileMode = Objects.requireNonNull(profileMode, "profileMode");
  }

  public void setOptions(RequestOptions options) {
    this.options = Objects.requireNonNull(options, "options");
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
    return switch (getOutputType()) {
      case CURRENT -> "PositionCurrent";
      case DUTY_CYCLE -> "PositionDutyCycle";
      case VOLTAGE -> "PositionVoltage";
    };
  }

  @Override
  public MotorRequest request() {
    return PositionRequest.of(
        profileMode,
        conversion.toMotorPosition(goal.getNewSetpoint()),
        feedforward,
        foc,
        getGainSlot(),
        options);
  }

  @Override
  public void periodic(String topic) {
    Logger.recordOutputMeasure(topic + "/Position/Goal", goal.getSetpoint());
    Logger.recordOutputMeasure(topic + "/Position/Offset", goal.getOffset());
    Logger.recordOutputMeasure(topic + "/Position/Measured", getPosition());
    Logger.recordOutput(topic + "/Position/AtGoal", atGoal());
    Logger.recordOutput(topic + "/Position/GainSlot", getGainSlot().name());
    Logger.recordOutput(topic + "/Position/OutputType", getOutputType().name());
  }
}
