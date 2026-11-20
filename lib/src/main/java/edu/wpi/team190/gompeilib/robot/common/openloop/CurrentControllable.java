package edu.wpi.team190.gompeilib.robot.common.openloop;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.robot.common.core.Mechanism;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.measure.Current;

/** A mechanism that can be driven open-loop at a torque current. */
public interface CurrentControllable extends Mechanism {
  /**
   * Returns the mechanism's current control.
   *
   * @return the control
   */
  OpenLoopControl<CurrentUnit> currentControl();

  default void setCurrentGoal(Current current) {
    currentControl().setGoal(current);
  }

  default void setCurrentGoal(Setpoint<CurrentUnit> goal) {
    currentControl().setGoal(goal);
  }

  default Setpoint<CurrentUnit> getCurrentGoal() {
    return currentControl().getGoal();
  }

  default boolean atCurrentGoal() {
    return currentControl().atGoal();
  }

  default boolean atCurrentGoal(Current reference) {
    return currentControl().atGoal(reference);
  }
}
