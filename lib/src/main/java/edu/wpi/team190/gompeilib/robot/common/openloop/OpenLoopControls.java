package edu.wpi.team190.gompeilib.robot.common.openloop;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Value;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismCore;
import java.util.Objects;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.DimensionlessUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Dimensionless;

/**
 * A mechanism's voltage, torque-current, and duty-cycle controls, built the same way for every
 * mechanism.
 *
 * @param voltage the voltage control
 * @param current the torque-current control
 * @param dutyCycle the duty-cycle control
 */
public record OpenLoopControls(
    OpenLoopControl<VoltageUnit> voltage,
    OpenLoopControl<CurrentUnit> current,
    OpenLoopControl<DimensionlessUnit> dutyCycle) {
  public OpenLoopControls {
    Objects.requireNonNull(voltage, "voltage");
    Objects.requireNonNull(current, "current");
    Objects.requireNonNull(dutyCycle, "dutyCycle");
  }

  /**
   * Creates the three controls and registers them with the core. Each uses the motor config's FOC
   * setting. The current goal starts at 0 A with its offset limited to ± the stator current limit,
   * when there is one; the duty-cycle goal starts at 0 with its offset limited to [-1, 1].
   *
   * @param core the mechanism's core
   * @param motor the leader's config
   * @param voltageGoal the initial voltage goal
   * @param currentStep how far one increment or decrement moves the current goal
   * @param dutyCycleStep how far one increment or decrement moves the duty-cycle goal
   * @return the controls
   */
  public static OpenLoopControls create(
      MechanismCore core,
      MotorConfig motor,
      Setpoint<VoltageUnit> voltageGoal,
      Current currentStep,
      Dimensionless dutyCycleStep) {
    boolean foc = motor.enableFOC;
    Current limit = motor.currentLimits.statorCurrentLimit();
    Setpoint<CurrentUnit> currentGoal =
        limit == null
            ? new Setpoint<>(Amps.of(0), currentStep)
            : new Setpoint<>(Amps.of(0), currentStep, limit.unaryMinus(), limit);
    return new OpenLoopControls(
        OpenLoopControl.voltage(core, voltageGoal, foc),
        OpenLoopControl.current(core, currentGoal),
        OpenLoopControl.dutyCycle(
            core, new Setpoint<>(Value.of(0), dutyCycleStep, Value.of(-1), Value.of(1)), foc));
  }
}
