package edu.wpi.team190.gompeilib.io.motor.config;

import edu.wpi.team190.gompeilib.io.base.Signal;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;

/**
 * The status signals {@link MotorTelemetry} is filled from, each in one {@link MotorSignalGroup}.
 */
public enum MotorSignal implements Signal<MotorSignalGroup> {
  /** {@link MotorTelemetry#position}. */
  POSITION(MotorSignalGroup.FEEDBACK),

  /** {@link MotorTelemetry#velocity}. */
  VELOCITY(MotorSignalGroup.FEEDBACK),

  /** {@link MotorTelemetry#acceleration}. */
  ACCELERATION(MotorSignalGroup.FEEDBACK),

  /** {@link MotorTelemetry#rotorPosition}. */
  ROTOR_POSITION(MotorSignalGroup.FEEDBACK),

  /** {@link MotorTelemetry#rotorVelocity}. */
  ROTOR_VELOCITY(MotorSignalGroup.FEEDBACK),

  /** {@link MotorTelemetry#appliedVoltage}. */
  APPLIED_VOLTAGE(MotorSignalGroup.OUTPUT),

  /** {@link MotorTelemetry#dutyCycle}. */
  DUTY_CYCLE(MotorSignalGroup.OUTPUT),

  /** {@link MotorTelemetry#torqueCurrent}. */
  TORQUE_CURRENT(MotorSignalGroup.OUTPUT),

  /** {@link MotorTelemetry#supplyVoltage}. */
  SUPPLY_VOLTAGE(MotorSignalGroup.OUTPUT),

  /** {@link MotorTelemetry#supplyCurrent}. */
  SUPPLY_CURRENT(MotorSignalGroup.CURRENT),

  /** {@link MotorTelemetry#outputCurrent}. */
  OUTPUT_CURRENT(MotorSignalGroup.CURRENT),

  /** {@link MotorTelemetry#forwardLimitSwitchAsserted}. */
  FORWARD_LIMIT_SWITCH_ASSERTED(MotorSignalGroup.LIMITS),

  /** {@link MotorTelemetry#reverseLimitSwitchAsserted}. */
  REVERSE_LIMIT_SWITCH_ASSERTED(MotorSignalGroup.LIMITS),

  /** {@link MotorTelemetry#controlMode}. */
  CONTROL_MODE(MotorSignalGroup.CONTROL_STATE),

  /** {@link MotorTelemetry#gainSlot}. */
  GAIN_SLOT(MotorSignalGroup.CONTROL_STATE),

  /** {@link MotorTelemetry#motorOutputStatus}. */
  MOTOR_OUTPUT_STATUS(MotorSignalGroup.CONTROL_STATE),

  /** {@link MotorTelemetry#bridgeOutput}. */
  BRIDGE_OUTPUT(MotorSignalGroup.CONTROL_STATE),

  /** {@link MotorTelemetry#robotEnabled}. */
  ROBOT_ENABLED(MotorSignalGroup.CONTROL_STATE),

  /** {@link MotorTelemetry#deviceEnabled}. */
  DEVICE_ENABLED(MotorSignalGroup.CONTROL_STATE),

  /**
   * {@link MotorTelemetry#closedLoopPositionReference} or {@link
   * MotorTelemetry#closedLoopVelocityReference}.
   */
  CLOSED_LOOP_REFERENCE(MotorSignalGroup.CLOSED_LOOP),

  /**
   * {@link MotorTelemetry#closedLoopPositionReferenceSlope} or {@link
   * MotorTelemetry#closedLoopVelocityReferenceSlope}.
   */
  CLOSED_LOOP_REFERENCE_SLOPE(MotorSignalGroup.CLOSED_LOOP),

  /**
   * {@link MotorTelemetry#closedLoopPositionError} or {@link
   * MotorTelemetry#closedLoopVelocityError}.
   */
  CLOSED_LOOP_ERROR(MotorSignalGroup.CLOSED_LOOP),

  /** {@link MotorTelemetry#closedLoopProportionalOutput}. */
  CLOSED_LOOP_PROPORTIONAL_OUTPUT(MotorSignalGroup.CLOSED_LOOP),

  /** {@link MotorTelemetry#closedLoopIntegratedOutput}. */
  CLOSED_LOOP_INTEGRATED_OUTPUT(MotorSignalGroup.CLOSED_LOOP),

  /** {@link MotorTelemetry#closedLoopDerivativeOutput}. */
  CLOSED_LOOP_DERIVATIVE_OUTPUT(MotorSignalGroup.CLOSED_LOOP),

  /** {@link MotorTelemetry#closedLoopFeedforward}. */
  CLOSED_LOOP_FEEDFORWARD(MotorSignalGroup.CLOSED_LOOP),

  /** {@link MotorTelemetry#closedLoopOutput}. */
  CLOSED_LOOP_OUTPUT(MotorSignalGroup.CLOSED_LOOP),

  /** {@link MotorTelemetry#profileAtTarget}. */
  PROFILE_AT_TARGET(MotorSignalGroup.CLOSED_LOOP),

  /** {@link MotorTelemetry#profileRunning}. */
  PROFILE_RUNNING(MotorSignalGroup.CLOSED_LOOP),

  /** {@link MotorTelemetry#deviceTemperature}. */
  DEVICE_TEMPERATURE(MotorSignalGroup.THERMAL),

  /** {@link MotorTelemetry#processorTemperature}. */
  PROCESSOR_TEMPERATURE(MotorSignalGroup.THERMAL),

  /** {@link MotorTelemetry#ancillaryDeviceTemperature}. */
  ANCILLARY_DEVICE_TEMPERATURE(MotorSignalGroup.THERMAL),

  /** {@link MotorTelemetry#firmwareVersion}. */
  FIRMWARE_VERSION(MotorSignalGroup.IDENTITY),

  /** {@link MotorTelemetry#isProLicensed}. */
  IS_PRO_LICENSED(MotorSignalGroup.IDENTITY),

  /** {@link MotorTelemetry#connectedMotor}. */
  CONNECTED_MOTOR(MotorSignalGroup.IDENTITY),

  /** {@link MotorTelemetry#rotorPolarity}. */
  ROTOR_POLARITY(MotorSignalGroup.IDENTITY),

  /** {@link MotorTelemetry#motorKT}. */
  MOTOR_KT(MotorSignalGroup.IDENTITY),

  /** {@link MotorTelemetry#motorKV}. */
  MOTOR_KV(MotorSignalGroup.IDENTITY),

  /** {@link MotorTelemetry#motorStallCurrent}. */
  MOTOR_STALL_CURRENT(MotorSignalGroup.IDENTITY),

  /** {@link MotorTelemetry#faults}. */
  FAULTS(MotorSignalGroup.FAULTS),

  /** {@link MotorTelemetry#stickyFaults}. */
  STICKY_FAULTS(MotorSignalGroup.FAULTS),

  /** {@link MotorTelemetry#differentialControlMode}. */
  DIFFERENTIAL_CONTROL_MODE(MotorSignalGroup.DIFFERENTIAL),

  /** {@link MotorTelemetry#differentialAveragePosition}. */
  DIFFERENTIAL_AVERAGE_POSITION(MotorSignalGroup.DIFFERENTIAL),

  /** {@link MotorTelemetry#differentialAverageVelocity}. */
  DIFFERENTIAL_AVERAGE_VELOCITY(MotorSignalGroup.DIFFERENTIAL),

  /** {@link MotorTelemetry#differentialDifferencePosition}. */
  DIFFERENTIAL_DIFFERENCE_POSITION(MotorSignalGroup.DIFFERENTIAL),

  /** {@link MotorTelemetry#differentialDifferenceVelocity}. */
  DIFFERENTIAL_DIFFERENCE_VELOCITY(MotorSignalGroup.DIFFERENTIAL),

  /** {@link MotorTelemetry#differentialGainSlot}. */
  DIFFERENTIAL_GAIN_SLOT(MotorSignalGroup.DIFFERENTIAL),

  /** {@link MotorTelemetry#differentialOutput}. */
  DIFFERENTIAL_OUTPUT(MotorSignalGroup.DIFFERENTIAL),

  /** {@link MotorTelemetry#differentialClosedLoopProportionalOutput}. */
  DIFFERENTIAL_CLOSED_LOOP_PROPORTIONAL_OUTPUT(MotorSignalGroup.DIFFERENTIAL),

  /** {@link MotorTelemetry#differentialClosedLoopIntegratedOutput}. */
  DIFFERENTIAL_CLOSED_LOOP_INTEGRATED_OUTPUT(MotorSignalGroup.DIFFERENTIAL),

  /** {@link MotorTelemetry#differentialClosedLoopDerivativeOutput}. */
  DIFFERENTIAL_CLOSED_LOOP_DERIVATIVE_OUTPUT(MotorSignalGroup.DIFFERENTIAL),

  /** {@link MotorTelemetry#differentialClosedLoopFeedforward}. */
  DIFFERENTIAL_CLOSED_LOOP_FEEDFORWARD(MotorSignalGroup.DIFFERENTIAL),

  /** {@link MotorTelemetry#differentialClosedLoopOutput}. */
  DIFFERENTIAL_CLOSED_LOOP_OUTPUT(MotorSignalGroup.DIFFERENTIAL),

  /**
   * {@link MotorTelemetry#differentialClosedLoopPositionReference} or {@link
   * MotorTelemetry#differentialClosedLoopVelocityReference}.
   */
  DIFFERENTIAL_CLOSED_LOOP_REFERENCE(MotorSignalGroup.DIFFERENTIAL),

  /**
   * {@link MotorTelemetry#differentialClosedLoopPositionReferenceSlope} or {@link
   * MotorTelemetry#differentialClosedLoopVelocityReferenceSlope}.
   */
  DIFFERENTIAL_CLOSED_LOOP_REFERENCE_SLOPE(MotorSignalGroup.DIFFERENTIAL),

  /**
   * {@link MotorTelemetry#differentialClosedLoopPositionError} or {@link
   * MotorTelemetry#differentialClosedLoopVelocityError}.
   */
  DIFFERENTIAL_CLOSED_LOOP_ERROR(MotorSignalGroup.DIFFERENTIAL);

  /** Every group at its default rate, with no signal overrides. */
  public static final SignalRates<MotorSignal, MotorSignalGroup> DEFAULT_RATES =
      SignalRates.defaults(MotorSignal.class, MotorSignalGroup.class);

  private final MotorSignalGroup group;

  MotorSignal(MotorSignalGroup group) {
    this.group = group;
  }

  @Override
  public MotorSignalGroup group() {
    return group;
  }
}
