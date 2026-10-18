package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLib;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.motor.outputs.VelocityRequest;
import java.util.Objects;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.measure.Current;

/**
 * Closed-loop velocity control, in the mechanism's velocity unit. The loop either drives a voltage
 * output or a torque-current output with a current feedforward, each in its own gain slot.
 *
 * @param <V> the mechanism's velocity unit
 */
public final class VelocityControl<V extends Unit> implements MechanismCore.Control {
  private final MechanismCore core;
  private final MechanismConversion<?, V, ?> conversion;
  private final Supplier<? extends Measure<V>> tolerance;
  private final boolean foc;
  private final GainSlot voltageSlot;
  private final GainSlot currentSlot;

  @Getter private Setpoint<V> goal;
  @Getter private Current feedforward = Amps.of(0.0);
  @Getter private OutputType outputType = OutputType.VOLTAGE;

  /**
   * Creates a velocity control and registers it with the core.
   *
   * @param core the mechanism's core
   * @param conversion the mechanism's conversion
   * @param goal the initial goal
   * @param tolerance how close the velocity must be to count as at the goal, read each check
   * @param foc whether voltage requests use field-oriented control
   * @param voltageSlot the gain slot for the voltage loop
   * @param currentSlot the gain slot for the torque-current loop
   */
  public VelocityControl(
      MechanismCore core,
      MechanismConversion<?, V, ?> conversion,
      Setpoint<V> goal,
      Supplier<? extends Measure<V>> tolerance,
      boolean foc,
      GainSlot voltageSlot,
      GainSlot currentSlot) {
    this.core = core;
    this.conversion = Objects.requireNonNull(conversion, "conversion");
    this.goal = Objects.requireNonNull(goal, "goal");
    this.tolerance = Objects.requireNonNull(tolerance, "tolerance");
    this.foc = foc;
    this.voltageSlot = Objects.requireNonNull(voltageSlot, "voltageSlot");
    this.currentSlot = Objects.requireNonNull(currentSlot, "currentSlot");
    core.add(this);
  }

  /**
   * Sets the goal, runs the voltage loop, and makes this the active control.
   *
   * @param velocity the goal
   */
  public void setGoal(Measure<V> velocity) {
    goal.setSetpoint(velocity);
    runVoltage();
  }

  /**
   * Replaces the goal, runs the voltage loop, and makes this the active control.
   *
   * @param goal the goal
   */
  public void setGoal(Setpoint<V> goal) {
    this.goal = Objects.requireNonNull(goal, "goal");
    runVoltage();
  }

  /**
   * Sets the goal, runs the torque-current loop with a feedforward, and makes this the active
   * control.
   *
   * @param velocity the goal
   * @param feedforward the torque-current feedforward
   */
  public void setGoal(Measure<V> velocity, Current feedforward) {
    goal.setSetpoint(velocity);
    runCurrent(feedforward);
  }

  /**
   * Replaces the goal, runs the torque-current loop with a feedforward, and makes this the active
   * control.
   *
   * @param goal the goal
   * @param feedforward the torque-current feedforward
   */
  public void setGoal(Setpoint<V> goal, Current feedforward) {
    this.goal = Objects.requireNonNull(goal, "goal");
    runCurrent(feedforward);
  }

  private void runVoltage() {
    outputType = OutputType.VOLTAGE;
    core.activate(this);
  }

  private void runCurrent(Current feedforward) {
    this.feedforward = Objects.requireNonNull(feedforward, "feedforward");
    outputType = OutputType.CURRENT;
    core.activate(this);
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
    return outputType == OutputType.CURRENT ? "VelocityCurrent" : "VelocityVoltage";
  }

  @Override
  public MotorRequest request() {
    var velocity = conversion.toMotorVelocity(goal.getNewSetpoint());
    var acceleration = RadiansPerSecondPerSecond.of(0.0);
    if (outputType == OutputType.CURRENT) {
      return new VelocityRequest.VelocityCurrent(
          velocity, acceleration, feedforward, currentSlot, RequestOptions.DEFAULT);
    }
    return new VelocityRequest.VelocityVoltage(
        velocity, acceleration, Volts.of(0.0), foc, voltageSlot, RequestOptions.DEFAULT);
  }

  @Override
  public void periodic(String topic) {
    Logger.recordOutputMeasure(topic + "/Velocity/Goal", goal.getSetpoint());
    Logger.recordOutputMeasure(topic + "/Velocity/Offset", goal.getOffset());
    Logger.recordOutputMeasure(topic + "/Velocity/Measured", getVelocity());
    Logger.recordOutputMeasure(topic + "/Velocity/Feedforward", feedforward);
    Logger.recordOutput(topic + "/Velocity/AtGoal", atGoal());
  }
}
