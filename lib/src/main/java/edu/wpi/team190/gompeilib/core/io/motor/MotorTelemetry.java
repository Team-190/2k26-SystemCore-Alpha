package edu.wpi.team190.gompeilib.core.io.motor;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.TorqueUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.*;

/**
 * Everything one physical motor controller reports. Positions, velocities, and accelerations are at
 * the gearbox output (after the gear ratio, before any mechanism conversion) unless the field is
 * prefixed {@code rotor}. Fields a device doesn't report stay at their defaults.
 */
@AutoLog
public class MotorTelemetry {
  /** Whether the device is connected and communicating. */
  public boolean connected = false;

  /** Whether the device has a Pro license. */
  public boolean isProLicensed = false;

  /** The device's packed firmware version. */
  public int firmwareVersion = 0;

  /** Position at the gearbox output. */
  public Angle position = Radians.of(0.0);

  /** Velocity at the gearbox output. */
  public AngularVelocity velocity = RadiansPerSecond.of(0.0);

  /** Acceleration at the gearbox output. */
  public AngularAcceleration acceleration = RadiansPerSecondPerSecond.of(0.0);

  /** Position of the motor's own rotor, before the gear ratio. */
  public Angle rotorPosition = Radians.of(0.0);

  /** Velocity of the motor's own rotor, before the gear ratio. */
  public AngularVelocity rotorVelocity = RadiansPerSecond.of(0.0);

  /**
   * Acceleration of the motor's own rotor, before the gear ratio. Stays at its default on devices
   * that don't report it.
   */
  public AngularAcceleration rotorAcceleration = RadiansPerSecondPerSecond.of(0.0);

  /** Voltage applied to the motor. */
  public Voltage appliedVoltage = Volts.of(0.0);

  /** Voltage supplied to the controller. */
  public Voltage supplyVoltage = Volts.of(0.0);

  /** Applied output as a fraction of supply voltage, in [-1, 1]. */
  public double dutyCycle = 0.0;

  /** Current drawn from the supply. */
  public Current supplyCurrent = Amps.of(0.0);

  /** Current through the motor windings (stator current). */
  public Current outputCurrent = Amps.of(0.0);

  /** Torque-producing component of the motor current. */
  public Current torqueCurrent = Amps.of(0.0);

  /** Temperature of the motor or its power stage. */
  public Temperature deviceTemperature = Celsius.of(0.0);

  /** Temperature of the controller's processor. */
  public Temperature processorTemperature = Celsius.of(0.0);

  /** Temperature of the controller's ancillary electronics. */
  public Temperature ancillaryDeviceTemperature = Celsius.of(0.0);

  /** Which rotor direction is reported as positive. */
  public RotorPolarity rotorPolarity = RotorPolarity.UNKNOWN;

  /** The motor model the controller detects. */
  public ConnectedMotor connectedMotor = ConnectedMotor.UNKNOWN;

  /** The motor's torque constant. */
  public Per<TorqueUnit, CurrentUnit> motorKT = NewtonMeters.per(Amp).ofNative(0.0);

  /** The motor's velocity constant. */
  public Per<AngularVelocityUnit, VoltageUnit> motorKV = RadiansPerSecond.per(Volt).ofNative(0.0);

  /** The motor's stall current at the current supply voltage. */
  public Current motorStallCurrent = Amps.of(0.0);

  /** Whether the robot is enabled, as seen by the device. */
  public boolean robotEnabled = false;

  /** Whether the device itself is enabled to drive output. */
  public boolean deviceEnabled = false;

  /** The control mode the device is running. */
  public MotorControlMode controlMode = MotorControlMode.UNKNOWN;

  /** Whether the active control mode uses field-oriented control. */
  public boolean focActive = false;

  /** What the output stage is doing. */
  public MotorOutputStatus motorOutputStatus = MotorOutputStatus.UNKNOWN;

  /** The drive mode the output bridge is applying. */
  public BridgeOutput bridgeOutput = BridgeOutput.UNKNOWN;

  /** The gain slot the active closed loop is using. */
  public GainSlot gainSlot = GainSlot.ZERO;

  /** Closed-loop position target. Only filled while closed-looping on position. */
  public Angle closedLoopPositionReference = Radians.of(0.0);

  /** Rate of change of the closed-loop position target. Only filled on position. */
  public AngularVelocity closedLoopPositionReferenceSlope = RadiansPerSecond.of(0.0);

  /** Closed-loop position error. Only filled while closed-looping on position. */
  public Angle closedLoopPositionError = Radians.of(0.0);

  /** Closed-loop velocity target. Only filled while closed-looping on velocity. */
  public AngularVelocity closedLoopVelocityReference = RadiansPerSecond.of(0.0);

  /** Rate of change of the closed-loop velocity target. Only filled on velocity. */
  public AngularAcceleration closedLoopVelocityReferenceSlope = RadiansPerSecondPerSecond.of(0.0);

  /** Closed-loop velocity error. Only filled while closed-looping on velocity. */
  public AngularVelocity closedLoopVelocityError = RadiansPerSecond.of(0.0);

  /** Proportional term of the closed-loop output, in the active output type's unit. */
  public double closedLoopProportionalOutput = 0.0;

  /** Integral term of the closed-loop output, in the active output type's unit. */
  public double closedLoopIntegratedOutput = 0.0;

  /** Derivative term of the closed-loop output, in the active output type's unit. */
  public double closedLoopDerivativeOutput = 0.0;

  /** Feedforward term of the closed-loop output, in the active output type's unit. */
  public double closedLoopFeedforward = 0.0;

  /** Total closed-loop output, in the active output type's unit. */
  public double closedLoopOutput = 0.0;

  /** Whether the active motion profile has reached its target. */
  public boolean profileAtTarget = false;

  /** Whether a motion profile is running. */
  public boolean profileRunning = false;

  /** The control mode the difference axis is running. */
  public MotorControlMode differentialControlMode = MotorControlMode.UNKNOWN;

  /** Whether the difference axis uses field-oriented control. */
  public boolean differentialFocActive = false;

  /** Average position of the differential pair. */
  public Angle differentialAveragePosition = Radians.of(0.0);

  /** Average velocity of the differential pair. */
  public AngularVelocity differentialAverageVelocity = RadiansPerSecond.of(0.0);

  /** Position difference across the differential pair. */
  public Angle differentialDifferencePosition = Radians.of(0.0);

  /** Velocity difference across the differential pair. */
  public AngularVelocity differentialDifferenceVelocity = RadiansPerSecond.of(0.0);

  /** The gain slot the difference axis is using. */
  public GainSlot differentialGainSlot = GainSlot.ZERO;

  /** Output of the difference axis, in the active output type's unit. */
  public double differentialOutput = 0.0;

  /** Proportional term of the difference axis's closed loop. */
  public double differentialClosedLoopProportionalOutput = 0.0;

  /** Integral term of the difference axis's closed loop. */
  public double differentialClosedLoopIntegratedOutput = 0.0;

  /** Derivative term of the difference axis's closed loop. */
  public double differentialClosedLoopDerivativeOutput = 0.0;

  /** Feedforward term of the difference axis's closed loop. */
  public double differentialClosedLoopFeedforward = 0.0;

  /** Total output of the difference axis's closed loop. */
  public double differentialClosedLoopOutput = 0.0;

  /** Difference-axis position target. Only filled while closed-looping on position. */
  public Angle differentialClosedLoopPositionReference = Radians.of(0.0);

  /** Rate of change of the difference-axis position target. */
  public AngularVelocity differentialClosedLoopPositionReferenceSlope = RadiansPerSecond.of(0.0);

  /** Difference-axis position error. Only filled while closed-looping on position. */
  public Angle differentialClosedLoopPositionError = Radians.of(0.0);

  /** Difference-axis velocity target. Only filled while closed-looping on velocity. */
  public AngularVelocity differentialClosedLoopVelocityReference = RadiansPerSecond.of(0.0);

  /** Rate of change of the difference-axis velocity target. */
  public AngularAcceleration differentialClosedLoopVelocityReferenceSlope =
      RadiansPerSecondPerSecond.of(0.0);

  /** Difference-axis velocity error. Only filled while closed-looping on velocity. */
  public AngularVelocity differentialClosedLoopVelocityError = RadiansPerSecond.of(0.0);

  /** Whether the forward limit switch is asserted. */
  public boolean forwardLimitSwitchAsserted = false;

  /** Whether the reverse limit switch is asserted. */
  public boolean reverseLimitSwitchAsserted = false;

  /** Live faults, packed per the vendor's fault enum. */
  public long faults = 0L;

  /** Sticky faults, packed per the vendor's fault enum. */
  public long stickyFaults = 0L;
}
