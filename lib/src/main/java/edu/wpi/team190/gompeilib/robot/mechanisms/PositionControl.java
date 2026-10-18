package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLib;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
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

/**
 * Closed-loop position control along a trapezoidal profile, in the mechanism's position unit. Goals
 * go through the mechanism's conversion to the motors; readings come back the same way.
 *
 * @param <P> the mechanism's position unit
 */
public final class PositionControl<P extends Unit> implements MechanismCore.Control {
  private final MechanismCore core;
  private final MechanismConversion<P, ?, ?> conversion;
  private final Supplier<? extends Measure<P>> tolerance;
  private final boolean foc;

  @Getter private Setpoint<P> goal;
  @Getter @Setter private GainSlot gainSlot = GainSlot.ZERO;

  /**
   * Creates a position control and registers it with the core.
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
    this.core = core;
    this.conversion = Objects.requireNonNull(conversion, "conversion");
    this.goal = Objects.requireNonNull(goal, "goal");
    this.tolerance = Objects.requireNonNull(tolerance, "tolerance");
    this.foc = foc;
    core.add(this);
  }

  /**
   * Sets the goal and makes this the active control.
   *
   * @param position the goal
   */
  public void setGoal(Measure<P> position) {
    goal.setSetpoint(position);
    core.activate(this);
  }

  /**
   * Replaces the goal and makes this the active control.
   *
   * @param goal the goal
   */
  public void setGoal(Setpoint<P> goal) {
    this.goal = Objects.requireNonNull(goal, "goal");
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
    return "Position";
  }

  @Override
  public MotorRequest request() {
    return new PositionRequest.ProfiledPositionVoltage(
        conversion.toMotorPosition(goal.getNewSetpoint()),
        Volts.of(0.0),
        foc,
        gainSlot,
        RequestOptions.DEFAULT);
  }

  @Override
  public void periodic(String topic) {
    Logger.recordOutputMeasure(topic + "/Position/Goal", goal.getSetpoint());
    Logger.recordOutputMeasure(topic + "/Position/Offset", goal.getOffset());
    Logger.recordOutputMeasure(topic + "/Position/Measured", getPosition());
    Logger.recordOutput(topic + "/Position/AtGoal", atGoal());
    Logger.recordOutput(topic + "/Position/GainSlot", gainSlot.name());
  }
}
