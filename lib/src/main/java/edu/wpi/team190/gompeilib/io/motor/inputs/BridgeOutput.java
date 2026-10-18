package edu.wpi.team190.gompeilib.io.motor.inputs;

/** The drive mode the motor controller's output bridge is applying. */
public enum BridgeOutput {
  UNKNOWN,
  COAST,
  BRAKE,
  TRAPEZOIDAL,
  FOC_TORQUE,
  TONE,
  FOC_EASY,
  FAULT_BRAKE,
  FAULT_COAST,
  ACTIVE_BRAKE,
  VARIABLE_BRAKE;
}
