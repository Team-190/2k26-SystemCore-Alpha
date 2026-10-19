package edu.wpi.team190.gompeilib.robot.mechanisms;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import org.wpilib.command2.Command;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

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

  default void setPositionGoal(Measure<P> position) {
    positionControl().setGoal(position);
  }

  default void setPositionGoal(Setpoint<P> goal) {
    positionControl().setGoal(goal);
  }

  default void setPositionGoal(Measure<P> position, Voltage feedforward) {
    positionControl().setGoal(position, feedforward);
  }

  default void setPositionGoal(Setpoint<P> goal, Voltage feedforward) {
    positionControl().setGoal(goal, feedforward);
  }

  default void setPositionGoal(Measure<P> position, Current feedforward) {
    positionControl().setGoal(position, feedforward);
  }

  default void setPositionGoal(Setpoint<P> goal, Current feedforward) {
    positionControl().setGoal(goal, feedforward);
  }

  default void setPositionGoal(Measure<P> position, double feedforward) {
    positionControl().setGoal(position, feedforward);
  }

  default void setPositionGoal(Setpoint<P> goal, double feedforward) {
    positionControl().setGoal(goal, feedforward);
  }

  default Setpoint<P> getPositionGoal() {
    return positionControl().getGoal();
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
   * Sets the gain slot position control uses from the next loop on.
   *
   * @param slot the slot
   */
  default void setPositionGainSlot(GainSlot slot) {
    positionControl().setGainSlot(slot);
  }

  default GainSlot getPositionGainSlot() {
    return positionControl().getGainSlot();
  }

  default void setPositionOutputType(OutputType outputType) {
    positionControl().setOutputType(outputType);
  }

  default OutputType getPositionOutputType() {
    return positionControl().getOutputType();
  }
}
