package edu.wpi.team190.gompeilib.io.motor.config;

import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import java.util.List;
import java.util.Map;
import java.util.Set;
import lombok.Builder;
import lombok.NonNull;
import lombok.Singular;

/**
 * Everything one motor controller needs, shared by every mechanism. Mechanism constants embed this
 * and keep only mechanism-specific values.
 */
@Builder(setterPrefix = "with", toBuilder = true)
public class MotorConfig {
  /**
   * The {@link edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry} fields published at the
   * control rate.
   */
  public static final Set<String> CONTROL_FIELDS =
      Set.of(
          "position",
          "velocity",
          "acceleration",
          "rotorPosition",
          "rotorVelocity",
          "appliedVoltage",
          "supplyVoltage",
          "dutyCycle",
          "supplyCurrent",
          "outputCurrent",
          "torqueCurrent",
          "controlMode",
          "focActive",
          "gainSlot",
          "closedLoopPositionReference",
          "closedLoopPositionReferenceSlope",
          "closedLoopPositionError",
          "closedLoopVelocityReference",
          "closedLoopVelocityReferenceSlope",
          "closedLoopVelocityError",
          "closedLoopProportionalOutput",
          "closedLoopIntegratedOutput",
          "closedLoopDerivativeOutput",
          "closedLoopFeedforward",
          "closedLoopOutput",
          "profileAtTarget",
          "profileRunning",
          "differentialControlMode",
          "differentialFocActive",
          "differentialAveragePosition",
          "differentialAverageVelocity",
          "differentialDifferencePosition",
          "differentialDifferenceVelocity",
          "differentialGainSlot",
          "differentialOutput",
          "differentialClosedLoopProportionalOutput",
          "differentialClosedLoopIntegratedOutput",
          "differentialClosedLoopDerivativeOutput",
          "differentialClosedLoopFeedforward",
          "differentialClosedLoopOutput",
          "differentialClosedLoopPositionReference",
          "differentialClosedLoopPositionReferenceSlope",
          "differentialClosedLoopPositionError",
          "differentialClosedLoopVelocityReference",
          "differentialClosedLoopVelocityReferenceSlope",
          "differentialClosedLoopVelocityError",
          "forwardLimitSwitchAsserted",
          "reverseLimitSwitchAsserted");

  /** The motor's CAN ID. */
  @NonNull public final Integer canId;

  /** The bus the motor is on. */
  @NonNull public final CANBusId bus;

  /** Which rotor direction is positive. */
  @NonNull @Builder.Default
  public final MotorDirection direction = MotorDirection.COUNTER_CLOCKWISE_POSITIVE;

  /** What the motor does at neutral. */
  @NonNull @Builder.Default public final NeutralMode neutralMode = NeutralMode.BRAKE;

  /** Supply and stator current limits. A null limit is disabled. */
  @NonNull public final CurrentLimits currentLimits;

  /** The peak torque-producing current in each direction. */
  @NonNull @Builder.Default
  public final TorqueCurrentLimits torqueCurrentLimits = TorqueCurrentLimits.NONE;

  /** Positions past which the motor won't drive further. */
  @NonNull @Builder.Default public final SoftLimits softLimits = SoftLimits.NONE;

  /** Rotor rotations per gearbox-output rotation. */
  @NonNull @Builder.Default public final Double gearRatio = 1.0;

  /** Where closed loops read position and velocity from. */
  @NonNull @Builder.Default public final FeedbackSensor feedbackSensor = FeedbackSensor.INTERNAL;

  /**
   * Whether position closed loops treat the gearbox output as continuous, taking the shortest path
   * to a target a whole number of rotations away.
   */
  @NonNull @Builder.Default public final Boolean continuousWrap = false;

  /** Closed-loop gains, by slot. Slots left out keep zero gains. */
  @NonNull @Singular public final Map<GainSlot, SlotConfig> slots;

  /** Constraints for profiled requests that don't carry their own. */
  @NonNull @Builder.Default
  public final MotionProfileConfig motionProfile = MotionProfileConfig.NONE;

  /** Open-loop ramp periods. */
  @NonNull @Builder.Default public final OpenLoopRamps openLoopRamps = OpenLoopRamps.NONE;

  /** Closed-loop ramp periods. */
  @NonNull @Builder.Default public final ClosedLoopRamps closedLoopRamps = ClosedLoopRamps.NONE;

  /** Whether requests this mechanism builds should use field-oriented control. */
  @NonNull @Builder.Default public final Boolean enableFOC = true;

  /** How often each telemetry field is published. */
  @NonNull @Builder.Default public final SignalRates signalRates = SignalRates.DEFAULT;

  /** Motors that follow this one. */
  @NonNull @Singular public final List<FollowerMotor> followers;
}
