package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.wpilib.units.Units.Value;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import java.util.Objects;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.units.DimensionlessUnit;
import org.wpilib.units.Measure;

/** Open-loop duty-cycle control: drives the motors at a duty-cycle goal in [-1, 1]. */
public final class DutyCycleControl implements MechanismCore.Control {
  /** How close the leader's duty cycle must be to count as at the goal. */
  public static final double TOLERANCE = 0.05;

  private final MechanismCore core;
  private final boolean foc;

  @Getter private Setpoint<DimensionlessUnit> goal;

  /**
   * Creates a duty cycle control and registers it with the core.
   *
   * @param core the mechanism's core
   * @param goal the initial goal
   * @param foc whether requests use field-oriented control
   */
  public DutyCycleControl(MechanismCore core, Setpoint<DimensionlessUnit> goal, boolean foc) {
    this.core = Objects.requireNonNull(core, "core");
    this.goal = Objects.requireNonNull(goal, "goal");
    this.foc = foc;
    core.add(this);
  }

  /**
   * Creates a duty cycle control with a 0 goal limited to [-1, 1], and registers it with the core.
   *
   * @param core the mechanism's core
   * @param offsetStep how far one increment or decrement moves the goal
   * @param foc whether requests use field-oriented control
   */
  public DutyCycleControl(MechanismCore core, double offsetStep, boolean foc) {
    this(core, new Setpoint<>(Value.of(0), Value.of(offsetStep), Value.of(-1), Value.of(1)), foc);
  }

  /**
   * Sets the goal and makes this the active control.
   *
   * @param dutyCycle the goal in [-1, 1]
   */
  public void setGoal(double dutyCycle) {
    goal.setSetpoint(Value.of(dutyCycle));
    core.activate(this);
  }

  /**
   * Sets the goal and makes this the active control.
   *
   * @param dutyCycle the goal
   */
  public void setGoal(Measure<DimensionlessUnit> dutyCycle) {
    goal.setSetpoint(Objects.requireNonNull(dutyCycle, "dutyCycle"));
    core.activate(this);
  }

  /**
   * Replaces the goal and makes this the active control.
   *
   * @param goal the goal
   */
  public void setGoal(Setpoint<DimensionlessUnit> goal) {
    this.goal = Objects.requireNonNull(goal, "goal");
    core.activate(this);
  }

  public boolean atGoal() {
    return atGoal(goal.getNewSetpoint().in(Value));
  }

  public boolean atGoal(double reference) {
    return Math.abs(core.getMotors().leader().dutyCycle - reference) <= TOLERANCE;
  }

  @Override
  public String name() {
    return "DutyCycle";
  }

  @Override
  public MotorRequest request() {
    return new OpenLoopRequest.DutyCycleOutput(
        goal.getNewSetpoint().in(Value), foc, RequestOptions.DEFAULT);
  }

  /**
   * Returns the request that drives the motors at a duty cycle, the way this control does.
   *
   * @param dutyCycle the duty cycle
   * @return the request
   */
  public MotorRequest request(double dutyCycle) {
    return new OpenLoopRequest.DutyCycleOutput(dutyCycle, foc, RequestOptions.DEFAULT);
  }

  /**
   * Returns the request that drives the motors at a duty cycle, the way this control does.
   *
   * @param dutyCycle the duty cycle
   * @return the request
   */
  public MotorRequest request(Measure<DimensionlessUnit> dutyCycle) {
    return new OpenLoopRequest.DutyCycleOutput(
        Objects.requireNonNull(dutyCycle, "dutyCycle").in(Value), foc, RequestOptions.DEFAULT);
  }

  @Override
  public void periodic(String topic) {
    Logger.recordOutputMeasure(topic + "/DutyCycle/Goal", goal.getSetpoint());
    Logger.recordOutputMeasure(topic + "/DutyCycle/Offset", goal.getOffset());
    Logger.recordOutput(topic + "/DutyCycle/AtGoal", atGoal());
  }
}
