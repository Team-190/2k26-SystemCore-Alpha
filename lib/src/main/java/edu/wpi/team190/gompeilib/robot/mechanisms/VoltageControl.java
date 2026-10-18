package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import java.util.Objects;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Voltage;

/** Open-loop voltage control: drives the motors at a voltage goal. */
public final class VoltageControl implements MechanismCore.Control {
  /** How close the leader's applied voltage must be to count as at the goal. */
  public static final Voltage TOLERANCE = Millivolts.of(500);

  private final MechanismCore core;
  private final boolean foc;

  @Getter private Setpoint<VoltageUnit> goal;

  /**
   * Creates a voltage control and registers it with the core.
   *
   * @param core the mechanism's core
   * @param goal the initial goal
   * @param foc whether requests use field-oriented control
   */
  public VoltageControl(MechanismCore core, Setpoint<VoltageUnit> goal, boolean foc) {
    this.core = core;
    this.goal = Objects.requireNonNull(goal, "goal");
    this.foc = foc;
    core.add(this);
  }

  /**
   * Creates a voltage control with a 0 V goal limited to ±12 V, and registers it with the core.
   *
   * @param core the mechanism's core
   * @param offsetStep how far one increment or decrement moves the goal
   * @param foc whether requests use field-oriented control
   */
  public VoltageControl(MechanismCore core, Voltage offsetStep, boolean foc) {
    this(core, new Setpoint<>(Volts.of(0), offsetStep, Volts.of(-12), Volts.of(12)), foc);
  }

  /**
   * Sets the goal and makes this the active control.
   *
   * @param voltage the goal
   */
  public void setGoal(Voltage voltage) {
    goal.setSetpoint(voltage);
    core.activate(this);
  }

  /**
   * Replaces the goal and makes this the active control.
   *
   * @param goal the goal
   */
  public void setGoal(Setpoint<VoltageUnit> goal) {
    this.goal = Objects.requireNonNull(goal, "goal");
    core.activate(this);
  }

  public boolean atGoal() {
    return atGoal((Voltage) goal.getNewSetpoint());
  }

  public boolean atGoal(Voltage reference) {
    return core.getMotors().leader().appliedVoltage.isNear(reference, TOLERANCE);
  }

  @Override
  public String name() {
    return "Voltage";
  }

  @Override
  public MotorRequest request() {
    return new OpenLoopRequest.VoltageOutput(
        (Voltage) goal.getNewSetpoint(), foc, RequestOptions.DEFAULT);
  }

  @Override
  public void periodic(String topic) {
    Logger.recordOutputMeasure(topic + "/Voltage/Goal", goal.getSetpoint());
    Logger.recordOutputMeasure(topic + "/Voltage/Offset", goal.getOffset());
    Logger.recordOutput(topic + "/Voltage/AtGoal", atGoal());
  }
}
