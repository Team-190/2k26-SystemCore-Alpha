package edu.wpi.team190.gompeilib.robot.mechanisms;

import edu.wpi.team190.gompeilib.control.Setpoint;
import org.wpilib.command2.Command;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.measure.Current;

/**
 * A mechanism that can hold a velocity goal in closed loop, on a voltage or torque-current output.
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
   * Sets the goal and runs the voltage loop.
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

  default Setpoint<V> getVelocityGoal() {
    return velocityControl().getGoal();
  }

  default Current getVelocityFeedforward() {
    return velocityControl().getFeedforward();
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
