package edu.wpi.team190.gompeilib.io.motor.phoenix;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.AppliedRotorPolarityValue;
import com.ctre.phoenix6.signals.BridgeOutputValue;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.DifferentialControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.FeedbackSensor;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.GravityType;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorAlignment;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorDirection;
import edu.wpi.team190.gompeilib.io.motor.config.NeutralMode;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.io.motor.inputs.BridgeOutput;
import edu.wpi.team190.gompeilib.io.motor.inputs.ConnectedMotor;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorControlMode;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorOutputStatus;
import edu.wpi.team190.gompeilib.io.motor.inputs.RotorPolarity;
import edu.wpi.team190.gompeilib.io.motor.outputs.Differential;
import edu.wpi.team190.gompeilib.io.motor.outputs.FollowerRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.NeutralRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.PositionRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.motor.outputs.VelocityRequest;
import java.util.Map;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.MultUnit;
import org.wpilib.units.PerUnit;
import org.wpilib.units.Unit;

/**
 * Pure conversions between GompeiLib's vendor-neutral motor types and Phoenix6's TalonFX types.
 * Kept separate from {@link MotorIOTalonFX} so every mapping is testable without hardware.
 */
public final class TalonFXConversions {
  private TalonFXConversions() {}

  /**
   * Builds the device configuration for a motor.
   *
   * @param config the motor's config
   * @return the TalonFX configuration
   */
  public static TalonFXConfiguration toConfiguration(MotorConfig config) {
    TalonFXConfiguration talon = new TalonFXConfiguration();
    talon.MotorOutput.Inverted =
        config.direction == MotorDirection.CLOCKWISE_POSITIVE
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    talon.MotorOutput.NeutralMode =
        config.neutralMode == NeutralMode.COAST ? NeutralModeValue.Coast : NeutralModeValue.Brake;

    talon.CurrentLimits = toCurrentLimitsConfigs(config.currentLimits);

    talon.TorqueCurrent.PeakForwardTorqueCurrent =
        config.torqueCurrentLimits.peakForward().in(Amps);
    talon.TorqueCurrent.PeakReverseTorqueCurrent =
        config.torqueCurrentLimits.peakReverse().in(Amps);

    config
        .softLimits
        .forward()
        .ifPresent(
            limit -> {
              talon.SoftwareLimitSwitch.ForwardSoftLimitThreshold = limit.in(Rotations);
              talon.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            });
    config
        .softLimits
        .reverse()
        .ifPresent(
            limit -> {
              talon.SoftwareLimitSwitch.ReverseSoftLimitThreshold = limit.in(Rotations);
              talon.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            });

    switch (config.feedbackSensor) {
      case FeedbackSensor.Internal internal ->
          talon.Feedback.SensorToMechanismRatio = config.gearRatio;
      case FeedbackSensor.RemoteEncoder remote -> {
        talon.Feedback.FeedbackRemoteSensorID = remote.canId();
        talon.Feedback.FeedbackSensorSource =
            switch (remote.mode()) {
              case REMOTE -> FeedbackSensorSourceValue.RemoteCANcoder;
              case FUSED -> FeedbackSensorSourceValue.FusedCANcoder;
              case SYNC -> FeedbackSensorSourceValue.SyncCANcoder;
            };
        talon.Feedback.RotorToSensorRatio = config.gearRatio;
        talon.Feedback.SensorToMechanismRatio = 1.0;
      }
    }
    talon.ClosedLoopGeneral.ContinuousWrap = config.continuousWrap;

    talon.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = config.openLoopRamps.dutyCycle().in(Seconds);
    talon.OpenLoopRamps.VoltageOpenLoopRampPeriod = config.openLoopRamps.voltage().in(Seconds);
    talon.OpenLoopRamps.TorqueOpenLoopRampPeriod = config.openLoopRamps.current().in(Seconds);
    talon.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
        config.closedLoopRamps.dutyCycle().in(Seconds);
    talon.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        config.closedLoopRamps.voltage().in(Seconds);
    talon.ClosedLoopRamps.TorqueClosedLoopRampPeriod = config.closedLoopRamps.current().in(Seconds);

    talon.MotionMagic = toMotionMagicConfigs(config.motionProfile);

    for (Map.Entry<GainSlot, SlotConfig> slot : config.slots.entrySet()) {
      SlotConfigs gains = toSlotConfigs(slot.getValue(), slot.getKey());
      if (slot.getKey() == GainSlot.ZERO) {
        talon.Slot0 = Slot0Configs.from(gains);
      } else if (slot.getKey() == GainSlot.ONE) {
        talon.Slot1 = Slot1Configs.from(gains);
      } else {
        talon.Slot2 = Slot2Configs.from(gains);
      }
    }
    return talon;
  }

  /**
   * Converts current limits. A null limit is disabled.
   *
   * @param limits the limits
   * @return the Phoenix6 current-limit configs
   */
  public static CurrentLimitsConfigs toCurrentLimitsConfigs(CurrentLimits limits) {
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
    if (limits.supplyCurrentLimit() != null) {
      configs.SupplyCurrentLimit = limits.supplyCurrentLimit().in(Amps);
    }
    configs.SupplyCurrentLimitEnable = limits.supplyCurrentLimit() != null;
    if (limits.statorCurrentLimit() != null) {
      configs.StatorCurrentLimit = limits.statorCurrentLimit().in(Amps);
    }
    configs.StatorCurrentLimitEnable = limits.statorCurrentLimit() != null;
    return configs;
  }

  /**
   * Converts motion-profile constraints.
   *
   * @param profile the constraints
   * @return the Phoenix6 Motion Magic configs
   */
  public static MotionMagicConfigs toMotionMagicConfigs(MotionProfileConfig profile) {
    MotionMagicConfigs configs = new MotionMagicConfigs();
    configs.MotionMagicCruiseVelocity = profile.cruiseVelocity().in(RotationsPerSecond);
    configs.MotionMagicAcceleration = profile.acceleration().in(RotationsPerSecondPerSecond);
    configs.MotionMagicJerk = profile.jerk().in(RotationsPerSecondPerSecond.per(Second));
    configs.MotionMagicExpo_kV = profile.exponentialKV().in(Volts.per(RotationsPerSecond));
    configs.MotionMagicExpo_kA = profile.exponentialKA().in(Volts.per(RotationsPerSecondPerSecond));
    return configs;
  }

  /**
   * Converts one slot's gains into Phoenix6's native slot units: the output's base unit (duty
   * cycle, volts, or amps) per gearbox-output rotation for a position slot, or per rotation per
   * second for a velocity slot. Every TalonFX slot goes through this.
   *
   * @param slotConfig the slot's gains
   * @param slot the slot
   * @return the Phoenix6 slot configs
   */
  public static SlotConfigs toSlotConfigs(SlotConfig slotConfig, GainSlot slot) {
    SlotConfigs configs =
        switch (slotConfig) {
          case SlotConfig.Position position -> toPositionSlot(position.gains());
          case SlotConfig.Velocity velocity -> toVelocitySlot(velocity.gains());
        };
    configs.SlotNumber = slot.ordinal();
    configs.GravityType =
        slotConfig.gravityType() == GravityType.COSINE
            ? GravityTypeValue.Arm_Cosine
            : GravityTypeValue.Elevator_Static;
    return configs;
  }

  private static <O extends Unit> SlotConfigs toPositionSlot(
      Gains<O, AngleUnit, AngularVelocityUnit> gains) {
    O output = gains.baseOutputUnit();
    SlotConfigs configs = new SlotConfigs();
    configs.kP = gains.getKP(PerUnit.combine(output, Rotations));
    configs.kI = gains.getKI(PerUnit.combine(output, MultUnit.combine(Rotations, Seconds)));
    configs.kD = gains.getKD(PerUnit.combine(output, RotationsPerSecond));
    feedforward(configs, gains, output);
    return configs;
  }

  private static <O extends Unit> SlotConfigs toVelocitySlot(
      Gains<O, AngularVelocityUnit, AngularVelocityUnit> gains) {
    O output = gains.baseOutputUnit();
    SlotConfigs configs = new SlotConfigs();
    configs.kP = gains.getKP(PerUnit.combine(output, RotationsPerSecond));
    configs.kI =
        gains.getKI(PerUnit.combine(output, MultUnit.combine(RotationsPerSecond, Seconds)));
    configs.kD = gains.getKD(PerUnit.combine(output, RotationsPerSecondPerSecond));
    feedforward(configs, gains, output);
    return configs;
  }

  private static <O extends Unit> void feedforward(
      SlotConfigs configs, Gains<O, ?, AngularVelocityUnit> gains, O output) {
    configs.kS = gains.getKS(output);
    configs.kV = gains.getKV(PerUnit.combine(output, RotationsPerSecond));
    configs.kA = gains.getKA(PerUnit.combine(output, RotationsPerSecondPerSecond));
    configs.kG = gains.getKG(output);
  }

  /**
   * Converts a motor request to the Phoenix6 request that runs it.
   *
   * <p>Differential control is not supported yet: {@link Differential} and the differential
   * follower requests throw. Running them needs a differential sensor config, differential follower
   * support in {@code MotorControllerGroup}, and verification on hardware with Phoenix Pro and a
   * CANivore.
   *
   * @param request the request
   * @return the Phoenix6 request
   * @throws UnsupportedOperationException for differential requests
   */
  public static ControlRequest toControlRequest(MotorRequest request) {
    return switch (request) {
      case OpenLoopRequest.DutyCycleOutput r ->
          options(new DutyCycleOut(r.output()).withEnableFOC(r.foc()), r.options());
      case OpenLoopRequest.VoltageOutput r ->
          options(new VoltageOut(r.output()).withEnableFOC(r.foc()), r.options());
      case OpenLoopRequest.CurrentOutput r ->
          options(
              new TorqueCurrentFOC(r.output())
                  .withMaxAbsDutyCycle(r.maxAbsDutyCycle())
                  .withDeadband(r.deadband()),
              r.options());
      case PositionRequest.PositionDutyCycle r ->
          options(
              new com.ctre.phoenix6.controls.PositionDutyCycle(r.position())
                  .withVelocity(r.velocity())
                  .withFeedForward(r.feedforward())
                  .withEnableFOC(r.foc())
                  .withSlot(r.slot().ordinal()),
              r.options());
      case PositionRequest.PositionVoltage r ->
          options(
              new com.ctre.phoenix6.controls.PositionVoltage(r.position())
                  .withVelocity(r.velocity())
                  .withFeedForward(r.feedforward())
                  .withEnableFOC(r.foc())
                  .withSlot(r.slot().ordinal()),
              r.options());
      case PositionRequest.PositionCurrent r ->
          options(
              new PositionTorqueCurrentFOC(r.position())
                  .withVelocity(r.velocity())
                  .withFeedForward(r.feedforward())
                  .withSlot(r.slot().ordinal()),
              r.options());
      case PositionRequest.ProfiledPositionDutyCycle r ->
          options(
              new MotionMagicDutyCycle(r.position())
                  .withFeedForward(r.feedforward())
                  .withEnableFOC(r.foc())
                  .withSlot(r.slot().ordinal()),
              r.options());
      case PositionRequest.ProfiledPositionVoltage r ->
          options(
              new MotionMagicVoltage(r.position())
                  .withFeedForward(r.feedforward())
                  .withEnableFOC(r.foc())
                  .withSlot(r.slot().ordinal()),
              r.options());
      case PositionRequest.ProfiledPositionCurrent r ->
          options(
              new MotionMagicTorqueCurrentFOC(r.position())
                  .withFeedForward(r.feedforward())
                  .withSlot(r.slot().ordinal()),
              r.options());
      case PositionRequest.ExponentialProfiledPositionDutyCycle r ->
          options(
              new MotionMagicExpoDutyCycle(r.position())
                  .withFeedForward(r.feedforward())
                  .withEnableFOC(r.foc())
                  .withSlot(r.slot().ordinal()),
              r.options());
      case PositionRequest.ExponentialProfiledPositionVoltage r ->
          options(
              new MotionMagicExpoVoltage(r.position())
                  .withFeedForward(r.feedforward())
                  .withEnableFOC(r.foc())
                  .withSlot(r.slot().ordinal()),
              r.options());
      case PositionRequest.ExponentialProfiledPositionCurrent r ->
          options(
              new MotionMagicExpoTorqueCurrentFOC(r.position())
                  .withFeedForward(r.feedforward())
                  .withSlot(r.slot().ordinal()),
              r.options());
      case PositionRequest.DynamicProfiledPositionDutyCycle r ->
          options(
              new DynamicMotionMagicDutyCycle(r.position(), r.velocity(), r.acceleration())
                  .withJerk(r.jerk())
                  .withFeedForward(r.feedforward())
                  .withEnableFOC(r.foc())
                  .withSlot(r.slot().ordinal()),
              r.options());
      case PositionRequest.DynamicProfiledPositionVoltage r ->
          options(
              new DynamicMotionMagicVoltage(r.position(), r.velocity(), r.acceleration())
                  .withJerk(r.jerk())
                  .withFeedForward(r.feedforward())
                  .withEnableFOC(r.foc())
                  .withSlot(r.slot().ordinal()),
              r.options());
      case PositionRequest.DynamicProfiledPositionCurrent r ->
          options(
              new DynamicMotionMagicTorqueCurrentFOC(r.position(), r.velocity(), r.acceleration())
                  .withJerk(r.jerk())
                  .withFeedForward(r.feedforward())
                  .withSlot(r.slot().ordinal()),
              r.options());
      case PositionRequest.DynamicExponentialProfiledPositionDutyCycle r ->
          options(
              new DynamicMotionMagicExpoDutyCycle(r.position(), r.kV(), r.kA())
                  .withVelocity(r.velocity())
                  .withFeedForward(r.feedforward())
                  .withEnableFOC(r.foc())
                  .withSlot(r.slot().ordinal()),
              r.options());
      case PositionRequest.DynamicExponentialProfiledPositionVoltage r ->
          options(
              new DynamicMotionMagicExpoVoltage(r.position(), r.kV(), r.kA())
                  .withVelocity(r.velocity())
                  .withFeedForward(r.feedforward())
                  .withEnableFOC(r.foc())
                  .withSlot(r.slot().ordinal()),
              r.options());
      case PositionRequest.DynamicExponentialProfiledPositionCurrent r ->
          options(
              new DynamicMotionMagicExpoTorqueCurrentFOC(r.position(), r.kV(), r.kA())
                  .withVelocity(r.velocity())
                  .withFeedForward(r.feedforward())
                  .withSlot(r.slot().ordinal()),
              r.options());
      case VelocityRequest.VelocityDutyCycle r ->
          options(
              new com.ctre.phoenix6.controls.VelocityDutyCycle(r.velocity())
                  .withAcceleration(r.acceleration())
                  .withFeedForward(r.feedforward())
                  .withEnableFOC(r.foc())
                  .withSlot(r.slot().ordinal()),
              r.options());
      case VelocityRequest.VelocityVoltage r ->
          options(
              new com.ctre.phoenix6.controls.VelocityVoltage(r.velocity())
                  .withAcceleration(r.acceleration())
                  .withFeedForward(r.feedforward())
                  .withEnableFOC(r.foc())
                  .withSlot(r.slot().ordinal()),
              r.options());
      case VelocityRequest.VelocityCurrent r ->
          options(
              new VelocityTorqueCurrentFOC(r.velocity())
                  .withAcceleration(r.acceleration())
                  .withFeedForward(r.feedforward())
                  .withSlot(r.slot().ordinal()),
              r.options());
      case VelocityRequest.ProfiledVelocityDutyCycle r ->
          options(
              new MotionMagicVelocityDutyCycle(r.velocity())
                  .withAcceleration(r.acceleration())
                  .withFeedForward(r.feedforward())
                  .withEnableFOC(r.foc())
                  .withSlot(r.slot().ordinal()),
              r.options());
      case VelocityRequest.ProfiledVelocityVoltage r ->
          options(
              new MotionMagicVelocityVoltage(r.velocity())
                  .withAcceleration(r.acceleration())
                  .withFeedForward(r.feedforward())
                  .withEnableFOC(r.foc())
                  .withSlot(r.slot().ordinal()),
              r.options());
      case VelocityRequest.ProfiledVelocityCurrent r ->
          options(
              new MotionMagicVelocityTorqueCurrentFOC(r.velocity())
                  .withAcceleration(r.acceleration())
                  .withFeedForward(r.feedforward())
                  .withSlot(r.slot().ordinal()),
              r.options());
      case Differential r -> throw unsupported(r);
      case FollowerRequest.Follow r ->
          new Follower(r.leaderId(), toAlignment(r.alignment()))
              .withUpdateFreqHz(r.updateFrequency());
      case FollowerRequest.StrictFollow r ->
          new StrictFollower(r.leaderId()).withUpdateFreqHz(r.updateFrequency());
      case FollowerRequest.DifferentialFollow r -> throw unsupported(r);
      case FollowerRequest.DifferentialStrictFollow r -> throw unsupported(r);
      case NeutralRequest.Neutral r ->
          new NeutralOut().withUseTimesync(r.useTimesync()).withUpdateFreqHz(r.updateFrequency());
      case NeutralRequest.Coast r ->
          new CoastOut().withUseTimesync(r.useTimesync()).withUpdateFreqHz(r.updateFrequency());
      case NeutralRequest.StaticBrake r ->
          new StaticBrake().withUseTimesync(r.useTimesync()).withUpdateFreqHz(r.updateFrequency());
      case NeutralRequest.Tone r ->
          new MusicTone(r.audioFrequency()).withUpdateFreqHz(r.updateFrequency());
    };
  }

  private static UnsupportedOperationException unsupported(MotorRequest request) {
    return new UnsupportedOperationException(
        request.getClass().getSimpleName()
            + " is not supported by MotorIOTalonFX yet: differential control needs a differential"
            + " sensor config and hardware verification");
  }

  /**
   * Applies the shared options to a request, whichever neutral override it supports.
   *
   * @param request a Phoenix6 request
   * @param options the options
   * @return the request
   * @throws IllegalStateException if the request lacks one of the options
   */
  static ControlRequest options(Object request, RequestOptions options) {
    try {
      Class<?> type = request.getClass();
      String override =
          hasMethod(type, "withOverrideCoastDurNeutral")
              ? "withOverrideCoastDurNeutral"
              : "withOverrideBrakeDurNeutral";
      type.getMethod(override, boolean.class).invoke(request, options.overrideNeutralMode());
      type.getMethod("withLimitForwardMotion", boolean.class)
          .invoke(request, options.limitForwardMotion());
      type.getMethod("withLimitReverseMotion", boolean.class)
          .invoke(request, options.limitReverseMotion());
      type.getMethod("withIgnoreHardwareLimits", boolean.class)
          .invoke(request, options.ignoreHardwareLimits());
      type.getMethod("withIgnoreSoftwareLimits", boolean.class)
          .invoke(request, options.ignoreSoftwareLimits());
      type.getMethod("withUseTimesync", boolean.class).invoke(request, options.useTimesync());
      return ((ControlRequest) request).withUpdateFreqHz(options.updateFrequency());
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException(request.getClass().getSimpleName() + " lacks an option", e);
    }
  }

  private static boolean hasMethod(Class<?> type, String name) {
    try {
      type.getMethod(name, boolean.class);
      return true;
    } catch (NoSuchMethodException e) {
      return false;
    }
  }

  /**
   * Converts a follower alignment.
   *
   * @param alignment the alignment
   * @return the Phoenix6 alignment
   */
  public static MotorAlignmentValue toAlignment(MotorAlignment alignment) {
    return alignment == MotorAlignment.ALIGNED
        ? MotorAlignmentValue.Aligned
        : MotorAlignmentValue.Opposed;
  }

  /**
   * Converts a reported control mode, ignoring whether it uses field-oriented control.
   *
   * @param mode the Phoenix6 control mode
   * @return the control mode
   */
  public static MotorControlMode toControlMode(ControlModeValue mode) {
    return switch (mode) {
      case DisabledOutput -> MotorControlMode.DISABLED;
      case NeutralOut -> MotorControlMode.NEUTRAL;
      case StaticBrake -> MotorControlMode.STATIC_BRAKE;
      case CoastOut -> MotorControlMode.COAST;
      case Follower -> MotorControlMode.FOLLOWER;
      case MusicTone -> MotorControlMode.TONE;
      case UnauthorizedDevice -> MotorControlMode.UNAUTHORIZED;
      case Reserved -> MotorControlMode.UNKNOWN;
      case DutyCycleOut, DutyCycleFOC -> MotorControlMode.DUTY_CYCLE_OUTPUT;
      case PositionDutyCycle, PositionDutyCycleFOC -> MotorControlMode.POSITION_DUTY_CYCLE;
      case VelocityDutyCycle, VelocityDutyCycleFOC -> MotorControlMode.VELOCITY_DUTY_CYCLE;
      case MotionMagicDutyCycle, MotionMagicDutyCycleFOC ->
          MotorControlMode.PROFILED_POSITION_DUTY_CYCLE;
      case MotionMagicVelocityDutyCycle, MotionMagicVelocityDutyCycleFOC ->
          MotorControlMode.PROFILED_VELOCITY_DUTY_CYCLE;
      case MotionMagicExpoDutyCycle, MotionMagicExpoDutyCycleFOC ->
          MotorControlMode.EXPONENTIAL_PROFILED_POSITION_DUTY_CYCLE;
      case VoltageOut, VoltageFOC -> MotorControlMode.VOLTAGE_OUTPUT;
      case PositionVoltage, PositionVoltageFOC -> MotorControlMode.POSITION_VOLTAGE;
      case VelocityVoltage, VelocityVoltageFOC -> MotorControlMode.VELOCITY_VOLTAGE;
      case MotionMagicVoltage, MotionMagicVoltageFOC -> MotorControlMode.PROFILED_POSITION_VOLTAGE;
      case MotionMagicVelocityVoltage, MotionMagicVelocityVoltageFOC ->
          MotorControlMode.PROFILED_VELOCITY_VOLTAGE;
      case MotionMagicExpoVoltage, MotionMagicExpoVoltageFOC ->
          MotorControlMode.EXPONENTIAL_PROFILED_POSITION_VOLTAGE;
      case TorqueCurrentFOC -> MotorControlMode.CURRENT_OUTPUT;
      case PositionTorqueCurrentFOC -> MotorControlMode.POSITION_CURRENT;
      case VelocityTorqueCurrentFOC -> MotorControlMode.VELOCITY_CURRENT;
      case MotionMagicTorqueCurrentFOC -> MotorControlMode.PROFILED_POSITION_CURRENT;
      case MotionMagicVelocityTorqueCurrentFOC -> MotorControlMode.PROFILED_VELOCITY_CURRENT;
      case MotionMagicExpoTorqueCurrentFOC ->
          MotorControlMode.EXPONENTIAL_PROFILED_POSITION_CURRENT;
    };
  }

  /**
   * Returns whether a reported control mode uses field-oriented control.
   *
   * @param mode the Phoenix6 control mode
   * @return true for FOC modes, including every torque-current mode
   */
  public static boolean isFOC(ControlModeValue mode) {
    return mode.name().endsWith("FOC");
  }

  /**
   * Converts a reported differential control mode, which shares its values with the regular one.
   *
   * @param mode the Phoenix6 differential control mode
   * @return the control mode
   */
  public static MotorControlMode toControlMode(DifferentialControlModeValue mode) {
    return toControlMode(ControlModeValue.valueOf(mode.value));
  }

  /**
   * Returns whether a reported differential control mode uses field-oriented control.
   *
   * @param mode the Phoenix6 differential control mode
   * @return true for FOC modes
   */
  public static boolean isFOC(DifferentialControlModeValue mode) {
    return isFOC(ControlModeValue.valueOf(mode.value));
  }

  /**
   * Returns whether a control mode closes the loop on position.
   *
   * @param mode the control mode
   * @return true for position and position-profile modes
   */
  public static boolean isPositionMode(MotorControlMode mode) {
    return mode.name().contains("POSITION");
  }

  /**
   * Returns whether a control mode closes the loop on velocity.
   *
   * @param mode the control mode
   * @return true for velocity and velocity-profile modes
   */
  public static boolean isVelocityMode(MotorControlMode mode) {
    return mode.name().contains("VELOCITY");
  }

  /**
   * Converts a reported rotor polarity.
   *
   * @param polarity the Phoenix6 polarity
   * @return the polarity
   */
  public static RotorPolarity toRotorPolarity(AppliedRotorPolarityValue polarity) {
    return switch (polarity) {
      case PositiveIsCounterClockwise -> RotorPolarity.COUNTER_CLOCKWISE_POSITIVE;
      case PositiveIsClockwise -> RotorPolarity.CLOCKWISE_POSITIVE;
    };
  }

  /**
   * Converts a reported output status.
   *
   * @param status the Phoenix6 status
   * @return the status
   */
  public static MotorOutputStatus toMotorOutputStatus(MotorOutputStatusValue status) {
    return switch (status) {
      case Unknown -> MotorOutputStatus.UNKNOWN;
      case Off -> MotorOutputStatus.OFF;
      case StaticBraking -> MotorOutputStatus.STATIC_BRAKING;
      case Motoring -> MotorOutputStatus.MOTORING;
      case DiscordantMotoring -> MotorOutputStatus.DISCORDANT_MOTORING;
      case RegenBraking -> MotorOutputStatus.REGEN_BRAKING;
    };
  }

  /**
   * Converts a reported bridge output.
   *
   * @param output the Phoenix6 bridge output
   * @return the bridge output
   */
  public static BridgeOutput toBridgeOutput(BridgeOutputValue output) {
    return switch (output) {
      case BridgeReq_Coast -> BridgeOutput.COAST;
      case BridgeReq_Brake -> BridgeOutput.BRAKE;
      case BridgeReq_Trapez -> BridgeOutput.TRAPEZOIDAL;
      case BridgeReq_FOCTorque -> BridgeOutput.FOC_TORQUE;
      case BridgeReq_MusicTone -> BridgeOutput.TONE;
      case BridgeReq_FOCEasy -> BridgeOutput.FOC_EASY;
      case BridgeReq_FaultBrake -> BridgeOutput.FAULT_BRAKE;
      case BridgeReq_FaultCoast -> BridgeOutput.FAULT_COAST;
      case BridgeReq_ActiveBrake -> BridgeOutput.ACTIVE_BRAKE;
      case BridgeReq_VariableBrake -> BridgeOutput.VARIABLE_BRAKE;
    };
  }

  /**
   * Converts a reported motor type.
   *
   * @param motor the Phoenix6 motor type
   * @return the motor type
   */
  public static ConnectedMotor toConnectedMotor(ConnectedMotorValue motor) {
    return switch (motor) {
      case Unknown -> ConnectedMotor.UNKNOWN;
      case Falcon500_Integrated -> ConnectedMotor.FALCON_500;
      case KrakenX60_Integrated -> ConnectedMotor.KRAKEN_X60;
      case KrakenX44_Integrated -> ConnectedMotor.KRAKEN_X44;
      case Minion_JST -> ConnectedMotor.MINION;
      case Brushed_AB -> ConnectedMotor.BRUSHED_AB;
      case Brushed_AC -> ConnectedMotor.BRUSHED_AC;
      case Brushed_BC -> ConnectedMotor.BRUSHED_BC;
      case NEO_JST -> ConnectedMotor.NEO;
      case NEO550_JST -> ConnectedMotor.NEO_550;
      case VORTEX_JST -> ConnectedMotor.VORTEX;
      case CustomBrushless -> ConnectedMotor.CUSTOM_BRUSHLESS;
    };
  }

  /**
   * Converts a reported gain slot.
   *
   * @param slot the slot number
   * @return the slot, or {@link GainSlot#ZERO} if out of range
   */
  public static GainSlot toGainSlot(int slot) {
    GainSlot converted = GainSlot.integerToGainSlot(slot);
    return converted == null ? GainSlot.ZERO : converted;
  }
}
