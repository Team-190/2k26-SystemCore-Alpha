package edu.wpi.team190.gompeilib.robot.mechanisms;

import edu.wpi.team190.gompeilib.control.Setpoint;
import org.wpilib.units.DimensionlessUnit;
import org.wpilib.units.Measure;

/** A mechanism that can be driven open-loop at a duty cycle. */
public interface DutyCycleControllable extends Mechanism {
  /**
   * Returns the mechanism's duty cycle control.
   *
   * @return the control
   */
  DutyCycleControl dutyCycleControl();

  default void setDutyCycleGoal(double dutyCycle) {
    dutyCycleControl().setGoal(dutyCycle);
  }

  default void setDutyCycleGoal(Measure<DimensionlessUnit> dutyCycle) {
    dutyCycleControl().setGoal(dutyCycle);
  }

  default void setDutyCycleGoal(Setpoint<DimensionlessUnit> goal) {
    dutyCycleControl().setGoal(goal);
  }

  default Setpoint<DimensionlessUnit> getDutyCycleGoal() {
    return dutyCycleControl().getGoal();
  }

  default boolean atDutyCycleGoal() {
    return dutyCycleControl().atGoal();
  }

  default boolean atDutyCycleGoal(double reference) {
    return dutyCycleControl().atGoal(reference);
  }
}
