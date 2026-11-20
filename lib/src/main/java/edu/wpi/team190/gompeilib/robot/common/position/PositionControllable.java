package edu.wpi.team190.gompeilib.robot.common.position;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.outputs.Feedforward;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.motor.outputs.ProfileMode;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.robot.common.core.Mechanism;
import org.wpilib.command2.Command;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;

/**
 * A mechanism that can hold a position goal in closed loop.
 *
 * @param <P> the mechanism's position unit
 */
public interface PositionControllable<P extends Unit> extends Mechanism {
  /**
   * Returns the mechanism's position control.
   *
   * @return the control
   */
  PositionControl<P> positionControl();

  /**
   * Sets the goal, keeping the feedforward and output type.
   *
   * @param position the goal
   */
  default void setPositionGoal(Measure<P> position) {
    positionControl().setGoal(position);
  }

  default void setPositionGoal(Setpoint<P> goal) {
    positionControl().setGoal(goal);
  }

  /**
   * Sets the goal and the feedforward, whose kind picks the output type.
   *
   * @param position the goal
   * @param feedforward the feedforward
   */
  default void setPositionGoal(Measure<P> position, Feedforward feedforward) {
    positionControl().setGoal(position, feedforward);
  }

  default void setPositionGoal(Setpoint<P> goal, Feedforward feedforward) {
    positionControl().setGoal(goal, feedforward);
  }

  default Setpoint<P> getPositionGoal() {
    return positionControl().getGoal();
  }

  default Feedforward getPositionFeedforward() {
    return positionControl().getFeedforward();
  }

  default boolean atPositionGoal() {
    return positionControl().atGoal();
  }

  default boolean atPositionGoal(Measure<P> reference) {
    return positionControl().atGoal(reference);
  }

  default Command waitUntilAtPositionGoal() {
    return positionControl().waitUntilAtGoal();
  }

  /**
   * Resets every motor's reported position.
   *
   * @param position the mechanism's position now
   */
  default void setPosition(Measure<P> position) {
    positionControl().setPosition(position);
  }

  /**
   * Uses one gain slot for every output type, from the next loop on.
   *
   * @param slot the slot
   */
  default void setPositionGainSlot(GainSlot slot) {
    positionControl().setGainSlot(slot);
  }

  default void setPositionGainSlot(OutputType outputType, GainSlot slot) {
    positionControl().setGainSlot(outputType, slot);
  }

  default GainSlot getPositionGainSlot() {
    return positionControl().getGainSlot();
  }

  /**
   * Switches the loop to an output type, with zero feedforward.
   *
   * @param outputType the output type
   */
  default void setPositionOutputType(OutputType outputType) {
    positionControl().setOutputType(outputType);
  }

  default OutputType getPositionOutputType() {
    return positionControl().getOutputType();
  }

  default void setPositionProfileMode(ProfileMode profileMode) {
    positionControl().setProfileMode(profileMode);
  }

  default ProfileMode getPositionProfileMode() {
    return positionControl().getProfileMode();
  }

  default void setPositionOptions(RequestOptions options) {
    positionControl().setOptions(options);
  }

  default RequestOptions getPositionOptions() {
    return positionControl().getOptions();
  }
}
