package edu.wpi.team190.gompeilib.robot.mechanisms;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import org.wpilib.command2.Command;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

/**
 * A mechanism with closed-loop differential control across two coupled axes: an average axis and a
 * difference axis.
 *
 * @param <AvgUnit> the average axis unit
 * @param <DiffUnit> the difference axis unit
 */
public interface DifferentialControllable<AvgUnit extends Unit, DiffUnit extends Unit>
    extends Mechanism {
  /**
   * Returns the mechanism's differential control.
   *
   * @return the differential control
   */
  DifferentialControl<AvgUnit, DiffUnit> differentialControl();

  /** Sets both average and difference goals. */
  default void setDifferentialGoals(Measure<AvgUnit> average, Measure<DiffUnit> difference) {
    differentialControl().setGoals(average, difference);
  }

  /** Sets both average and difference goals. */
  default void setDifferentialGoals(Setpoint<AvgUnit> average, Setpoint<DiffUnit> difference) {
    differentialControl().setGoals(average, difference);
  }

  /** Sets the average goal. */
  default void setAverageGoal(Measure<AvgUnit> average) {
    differentialControl().setAverageGoal(average);
  }

  /** Sets the average goal. */
  default void setAverageGoal(Setpoint<AvgUnit> average) {
    differentialControl().setAverageGoal(average);
  }

  /** Sets the average goal with voltage feedforward. */
  default void setAverageGoal(Measure<AvgUnit> average, Voltage feedforward) {
    differentialControl().setAverageGoal(average, feedforward);
  }

  /** Sets the average goal with current feedforward. */
  default void setAverageGoal(Measure<AvgUnit> average, Current feedforward) {
    differentialControl().setAverageGoal(average, feedforward);
  }

  /** Sets the average goal with duty-cycle feedforward. */
  default void setAverageGoal(Measure<AvgUnit> average, double feedforward) {
    differentialControl().setAverageGoal(average, feedforward);
  }

  /** Sets the difference goal. */
  default void setDifferenceGoal(Measure<DiffUnit> difference) {
    differentialControl().setDifferenceGoal(difference);
  }

  /** Sets the difference goal. */
  default void setDifferenceGoal(Setpoint<DiffUnit> difference) {
    differentialControl().setDifferenceGoal(difference);
  }

  /** Sets the difference goal with voltage feedforward. */
  default void setDifferenceGoal(Measure<DiffUnit> difference, Voltage feedforward) {
    differentialControl().setDifferenceGoal(difference, feedforward);
  }

  /** Sets the difference goal with current feedforward. */
  default void setDifferenceGoal(Measure<DiffUnit> difference, Current feedforward) {
    differentialControl().setDifferenceGoal(difference, feedforward);
  }

  /** Sets the difference goal with duty-cycle feedforward. */
  default void setDifferenceGoal(Measure<DiffUnit> difference, double feedforward) {
    differentialControl().setDifferenceGoal(difference, feedforward);
  }

  /** Returns the current average goal. */
  default Setpoint<AvgUnit> getAverageGoal() {
    return differentialControl().getAverageGoal();
  }

  /** Returns the current difference goal. */
  default Setpoint<DiffUnit> getDifferenceGoal() {
    return differentialControl().getDifferenceGoal();
  }

  /** Checks if the average axis is at its goal. */
  default boolean atAverageGoal() {
    return differentialControl().atAverageGoal();
  }

  /** Checks if the difference axis is at its goal. */
  default boolean atDifferenceGoal() {
    return differentialControl().atDifferenceGoal();
  }

  /** Checks if both axes are at their goals. */
  default boolean atDifferentialGoals() {
    return differentialControl().atGoals();
  }

  /** Returns a command that waits until both differential goals are reached. */
  default Command waitUntilAtDifferentialGoals() {
    return differentialControl().waitUntilAtGoals();
  }

  /** Sets the closed-loop output type. */
  default void setDifferentialOutputType(OutputType outputType) {
    differentialControl().setOutputType(outputType);
  }

  /** Returns the closed-loop output type. */
  default OutputType getDifferentialOutputType() {
    return differentialControl().getOutputType();
  }
}
