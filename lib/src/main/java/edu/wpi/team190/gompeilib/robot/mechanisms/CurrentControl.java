package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import java.util.Objects;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.measure.Current;

/** Open-loop torque-current control: drives the motors at a torque-current goal. */
public final class CurrentControl implements MechanismCore.Control {
  /** How close the leader's torque current must be to count as at the goal. */
  public static final Current TOLERANCE = Milliamps.of(500);

  private final MechanismCore core;

  @Getter private Setpoint<CurrentUnit> goal;

  /**
   * Creates a current control and registers it with the core.
   *
   * @param core the mechanism's core
   * @param goal the initial goal
   */
  public CurrentControl(MechanismCore core, Setpoint<CurrentUnit> goal) {
    this.core = core;
    this.goal = Objects.requireNonNull(goal, "goal");
    core.add(this);
  }

  /**
   * Sets the goal and makes this the active control.
   *
   * @param current the goal
   */
  public void setGoal(Current current) {
    goal.setSetpoint(current);
    core.activate(this);
  }

  /**
   * Replaces the goal and makes this the active control.
   *
   * @param goal the goal
   */
  public void setGoal(Setpoint<CurrentUnit> goal) {
    this.goal = Objects.requireNonNull(goal, "goal");
    core.activate(this);
  }

  public boolean atGoal() {
    return atGoal((Current) goal.getNewSetpoint());
  }

  public boolean atGoal(Current reference) {
    return core.getMotors().leader().torqueCurrent.isNear(reference, TOLERANCE);
  }

  @Override
  public String name() {
    return "Current";
  }

  @Override
  public MotorRequest request() {
    return new OpenLoopRequest.CurrentOutput((Current) goal.getNewSetpoint());
  }

  /**
   * Returns the request that drives the motors at a torque current, the way this control does.
   *
   * @param current the torque current
   * @return the request
   */
  public MotorRequest request(Measure<CurrentUnit> current) {
    return new OpenLoopRequest.CurrentOutput(Amps.of(current.in(Amps)));
  }

  @Override
  public void periodic(String topic) {
    Logger.recordOutputMeasure(topic + "/Current/Goal", goal.getSetpoint());
    Logger.recordOutputMeasure(topic + "/Current/Offset", goal.getOffset());
    Logger.recordOutput(topic + "/Current/AtGoal", atGoal());
  }
}
