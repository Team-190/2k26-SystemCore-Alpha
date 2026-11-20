package edu.wpi.team190.gompeilib.robot.common.openloop;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.robot.common.core.Mechanism;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Voltage;

/** A mechanism that can be driven open-loop at a voltage. */
public interface VoltageControllable extends Mechanism {
  /**
   * Returns the mechanism's voltage control.
   *
   * @return the control
   */
  OpenLoopControl<VoltageUnit> voltageControl();

  default void setVoltageGoal(Voltage voltage) {
    voltageControl().setGoal(voltage);
  }

  default void setVoltageGoal(Setpoint<VoltageUnit> goal) {
    voltageControl().setGoal(goal);
  }

  default Setpoint<VoltageUnit> getVoltageGoal() {
    return voltageControl().getGoal();
  }

  default boolean atVoltageGoal() {
    return voltageControl().atGoal();
  }

  default boolean atVoltageGoal(Voltage reference) {
    return voltageControl().atGoal(reference);
  }
}
