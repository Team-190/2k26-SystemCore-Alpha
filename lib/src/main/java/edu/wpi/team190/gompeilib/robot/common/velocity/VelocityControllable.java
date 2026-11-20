package edu.wpi.team190.gompeilib.robot.common.velocity;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.outputs.Feedforward;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.motor.outputs.VelocityProfileMode;
import edu.wpi.team190.gompeilib.robot.common.core.Mechanism;
import org.wpilib.command2.Command;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;

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
   * Sets the goal, keeping the feedforward and output type.
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
   * Sets the goal and the feedforward, whose kind picks the output type.
   *
   * @param velocity the goal
   * @param feedforward the feedforward
   */
  default void setVelocityGoal(Measure<V> velocity, Feedforward feedforward) {
    velocityControl().setGoal(velocity, feedforward);
  }

  default void setVelocityGoal(Setpoint<V> goal, Feedforward feedforward) {
    velocityControl().setGoal(goal, feedforward);
  }

  default Setpoint<V> getVelocityGoal() {
    return velocityControl().getGoal();
  }

  default Feedforward getVelocityFeedforward() {
    return velocityControl().getFeedforward();
  }

  /**
   * Switches the loop to an output type, with zero feedforward.
   *
   * @param outputType the output type
   */
  default void setVelocityOutputType(OutputType outputType) {
    velocityControl().setOutputType(outputType);
  }

  default OutputType getVelocityOutputType() {
    return velocityControl().getOutputType();
  }

  /**
   * Uses one gain slot for every output type, from the next loop on.
   *
   * @param slot the slot
   */
  default void setVelocityGainSlot(GainSlot slot) {
    velocityControl().setGainSlot(slot);
  }

  default void setVelocityGainSlot(OutputType outputType, GainSlot slot) {
    velocityControl().setGainSlot(outputType, slot);
  }

  default GainSlot getVelocityGainSlot() {
    return velocityControl().getGainSlot();
  }

  default void setVelocityProfileMode(VelocityProfileMode profileMode) {
    velocityControl().setProfileMode(profileMode);
  }

  default VelocityProfileMode getVelocityProfileMode() {
    return velocityControl().getProfileMode();
  }

  default void setVelocityOptions(RequestOptions options) {
    velocityControl().setOptions(options);
  }

  default RequestOptions getVelocityOptions() {
    return velocityControl().getOptions();
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
