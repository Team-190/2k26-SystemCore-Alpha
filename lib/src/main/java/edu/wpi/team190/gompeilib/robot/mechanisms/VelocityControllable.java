package edu.wpi.team190.gompeilib.robot.mechanisms;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import org.wpilib.command2.Command;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

/**
 * A mechanism that can hold a velocity goal in closed loop, on a voltage, torque-current, or
 * duty-cycle output.
 *
 * @param <V> the mechanism's velocity unit
 */
public interface VelocityControllable<V extends Unit> extends Mechanism {
  /**
   * Returns the mechanism's velocity control.
   *
   * @return the control
   */
  VelocityControl<V> velocityControl();

  /**
   * Sets the goal, preserving the active output type.
   *
   * @param velocity the goal
   */
  default void setVelocityGoal(Measure<V> velocity) {
    velocityControl().setGoal(velocity);
  }

  default void setVelocityGoal(Setpoint<V> goal) {
    velocityControl().setGoal(goal);
  }

  /**
   * Sets the goal and runs the voltage loop with a feedforward.
   *
   * @param velocity the goal
   * @param feedforward the voltage feedforward
   */
  default void setVelocityGoal(Measure<V> velocity, Voltage feedforward) {
    velocityControl().setGoal(velocity, feedforward);
  }

  default void setVelocityGoal(Setpoint<V> goal, Voltage feedforward) {
    velocityControl().setGoal(goal, feedforward);
  }

  /**
   * Sets the goal and runs the torque-current loop with a feedforward.
   *
   * @param velocity the goal
   * @param feedforward the torque-current feedforward
   */
  default void setVelocityGoal(Measure<V> velocity, Current feedforward) {
    velocityControl().setGoal(velocity, feedforward);
  }

  default void setVelocityGoal(Setpoint<V> goal, Current feedforward) {
    velocityControl().setGoal(goal, feedforward);
  }

  /**
   * Sets the goal and runs the duty-cycle loop with a feedforward.
   *
   * @param velocity the goal
   * @param feedforward the duty-cycle feedforward in [-1, 1]
   */
  default void setVelocityGoal(Measure<V> velocity, double feedforward) {
    velocityControl().setGoal(velocity, feedforward);
  }

  default void setVelocityGoal(Setpoint<V> goal, double feedforward) {
    velocityControl().setGoal(goal, feedforward);
  }

  default Setpoint<V> getVelocityGoal() {
    return velocityControl().getGoal();
  }

  default Current getVelocityFeedforward() {
    return velocityControl().getFeedforward();
  }

  default Voltage getVelocityVoltageFeedforward() {
    return velocityControl().getVoltageFeedforward();
  }

  default double getVelocityDutyCycleFeedforward() {
    return velocityControl().getDutyCycleFeedforward();
  }

  default void setVelocityOutputType(OutputType outputType) {
    velocityControl().setOutputType(outputType);
  }

  default OutputType getVelocityOutputType() {
    return velocityControl().getOutputType();
  }

  default boolean atVelocityGoal() {
    return velocityControl().atGoal();
  }

  default boolean atVelocityGoal(Measure<V> reference) {
    return velocityControl().atGoal(reference);
  }

  default Command waitUntilAtVelocityGoal() {
    return velocityControl().waitUntilAtGoal();
  }
}
