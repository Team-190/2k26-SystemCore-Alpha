package edu.wpi.team190.gompeilib.io.motor.phoenix;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AppliedRotorPolarityValue;
import com.ctre.phoenix6.signals.BridgeOutputValue;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.DeviceEnableValue;
import com.ctre.phoenix6.signals.DifferentialControlModeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.signals.RobotEnableValue;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorSignal;
import edu.wpi.team190.gompeilib.io.motor.config.MotorSignalGroup;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.phoenix.CANBuses;
import edu.wpi.team190.gompeilib.io.phoenix.FaultSignals;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixOdometryThread;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixUtil;
import edu.wpi.team190.gompeilib.io.phoenix.SignalTracker;
import edu.wpi.team190.gompeilib.logging.Trace;
import java.util.Queue;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.TorqueUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.*;

/** A TalonFX motor controller. Logs every status signal the device publishes. */
public class MotorIOTalonFX implements MotorIO {
  /** The controller family TalonFX motors belong to, for hardware following. */
  public static final String FAMILY = "TalonFX";

  /** Builds TalonFX motors. */
  public static final MotorIOFactory FACTORY =
      new MotorIOFactory() {
        @Override
        public MotorIO create(MotorConfig config) {
          return new MotorIOTalonFX(config);
        }

        @Override
        public String family() {
          return FAMILY;
        }

        @Override
        public boolean supportsHardwareFollowing() {
          return true;
        }
      };

  protected final TalonFX talonFX;
  protected final MotorConfig config;

  private final SignalTracker<MotorSignal, MotorSignalGroup> signals;

  private final StatusSignal<Integer> version;
  private final StatusSignal<Boolean> isProLicensed;
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Angle> rotorPosition;
  private final StatusSignal<AngularVelocity> rotorVelocity;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Voltage> supplyVoltage;
  private final StatusSignal<Double> dutyCycle;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> deviceTemp;
  private final StatusSignal<Temperature> processorTemp;
  private final StatusSignal<Temperature> ancillaryDeviceTemp;
  private final StatusSignal<AppliedRotorPolarityValue> rotorPolarity;
  private final StatusSignal<ConnectedMotorValue> connectedMotor;
  private final StatusSignal<Per<TorqueUnit, CurrentUnit>> motorKT;
  private final StatusSignal<Per<AngularVelocityUnit, VoltageUnit>> motorKV;
  private final StatusSignal<Current> motorStallCurrent;
  private final StatusSignal<RobotEnableValue> robotEnable;
  private final StatusSignal<DeviceEnableValue> deviceEnable;
  private final StatusSignal<ControlModeValue> controlMode;
  private final StatusSignal<MotorOutputStatusValue> motorOutputStatus;
  private final StatusSignal<BridgeOutputValue> bridgeOutput;
  private final StatusSignal<Integer> closedLoopSlot;
  private final StatusSignal<Double> closedLoopReference;
  private final StatusSignal<Double> closedLoopReferenceSlope;
  private final StatusSignal<Double> closedLoopError;
  private final StatusSignal<Double> closedLoopProportionalOutput;
  private final StatusSignal<Double> closedLoopIntegratedOutput;
  private final StatusSignal<Double> closedLoopDerivativeOutput;
  private final StatusSignal<Double> closedLoopFeedForward;
  private final StatusSignal<Double> closedLoopOutput;
  private final StatusSignal<Boolean> motionMagicAtTarget;
  private final StatusSignal<Boolean> motionMagicIsRunning;
  private final StatusSignal<DifferentialControlModeValue> differentialControlMode;
  private final StatusSignal<Angle> differentialAveragePosition;
  private final StatusSignal<AngularVelocity> differentialAverageVelocity;
  private final StatusSignal<Angle> differentialDifferencePosition;
  private final StatusSignal<AngularVelocity> differentialDifferenceVelocity;
  private final StatusSignal<Integer> differentialClosedLoopSlot;
  private final StatusSignal<Double> differentialOutput;
  private final StatusSignal<Double> differentialClosedLoopProportionalOutput;
  private final StatusSignal<Double> differentialClosedLoopIntegratedOutput;
  private final StatusSignal<Double> differentialClosedLoopDerivativeOutput;
  private final StatusSignal<Double> differentialClosedLoopFeedForward;
  private final StatusSignal<Double> differentialClosedLoopOutput;
  private final StatusSignal<Double> differentialClosedLoopReference;
  private final StatusSignal<Double> differentialClosedLoopReferenceSlope;
  private final StatusSignal<Double> differentialClosedLoopError;
  private final StatusSignal<ForwardLimitValue> forwardLimit;
  private final StatusSignal<ReverseLimitValue> reverseLimit;
  private final FaultSignals<TalonFXFault> faults;

  private final Queue<Double> odometryTimestamps;
  private final Queue<Double> odometryPositionRotations;

  private boolean connected = false;

  /**
   * Creates and configures a TalonFX without odometry sampling.
   *
   * @param config the motor's config
   */
  public MotorIOTalonFX(MotorConfig config) {
    this(config, null);
  }

  /**
   * Creates and configures a TalonFX whose position is also sampled by an odometry thread. Publish
   * {@code position} at the odometry rate through {@link MotorConfig#signalRates} so every sample
   * is fresh.
   *
   * @param config the motor's config
   * @param odometryThread the thread that samples position between cycles, or null for none
   */
  public MotorIOTalonFX(MotorConfig config, PhoenixOdometryThread odometryThread) {
    this.config = config;
    CANBus canBus = CANBuses.toCANBus(config.bus);
    talonFX = new TalonFX(config.canId, canBus);
    signals = new SignalTracker<>(config.signalRates);

    PhoenixUtil.tryUntilOk(
        5, () -> talonFX.getConfigurator().apply(TalonFXConversions.toConfiguration(config), 0.25));

    version = track(MotorSignal.FIRMWARE_VERSION, talonFX.getVersion());
    isProLicensed = track(MotorSignal.IS_PRO_LICENSED, talonFX.getIsProLicensed());
    position = track(MotorSignal.POSITION, talonFX.getPosition());
    velocity = track(MotorSignal.VELOCITY, talonFX.getVelocity());
    acceleration = track(MotorSignal.ACCELERATION, talonFX.getAcceleration());
    rotorPosition = track(MotorSignal.ROTOR_POSITION, talonFX.getRotorPosition());
    rotorVelocity = track(MotorSignal.ROTOR_VELOCITY, talonFX.getRotorVelocity());
    motorVoltage = track(MotorSignal.APPLIED_VOLTAGE, talonFX.getMotorVoltage());
    supplyVoltage = track(MotorSignal.SUPPLY_VOLTAGE, talonFX.getSupplyVoltage());
    dutyCycle = track(MotorSignal.DUTY_CYCLE, talonFX.getDutyCycle());
    supplyCurrent = track(MotorSignal.SUPPLY_CURRENT, talonFX.getSupplyCurrent());
    statorCurrent = track(MotorSignal.OUTPUT_CURRENT, talonFX.getStatorCurrent());
    torqueCurrent = track(MotorSignal.TORQUE_CURRENT, talonFX.getTorqueCurrent());
    deviceTemp = track(MotorSignal.DEVICE_TEMPERATURE, talonFX.getDeviceTemp());
    processorTemp = track(MotorSignal.PROCESSOR_TEMPERATURE, talonFX.getProcessorTemp());
    ancillaryDeviceTemp =
        track(MotorSignal.ANCILLARY_DEVICE_TEMPERATURE, talonFX.getAncillaryDeviceTemp());
    rotorPolarity = track(MotorSignal.ROTOR_POLARITY, talonFX.getAppliedRotorPolarity());
    connectedMotor = track(MotorSignal.CONNECTED_MOTOR, talonFX.getConnectedMotor());
    motorKT = track(MotorSignal.MOTOR_KT, talonFX.getMotorKT());
    motorKV = track(MotorSignal.MOTOR_KV, talonFX.getMotorKV());
    motorStallCurrent = track(MotorSignal.MOTOR_STALL_CURRENT, talonFX.getMotorStallCurrent());
    robotEnable = track(MotorSignal.ROBOT_ENABLED, talonFX.getRobotEnable());
    deviceEnable = track(MotorSignal.DEVICE_ENABLED, talonFX.getDeviceEnable());
    controlMode = track(MotorSignal.CONTROL_MODE, talonFX.getControlMode());
    motorOutputStatus = track(MotorSignal.MOTOR_OUTPUT_STATUS, talonFX.getMotorOutputStatus());
    bridgeOutput = track(MotorSignal.BRIDGE_OUTPUT, talonFX.getBridgeOutput());
    closedLoopSlot = track(MotorSignal.GAIN_SLOT, talonFX.getClosedLoopSlot());
    closedLoopReference =
        track(MotorSignal.CLOSED_LOOP_REFERENCE, talonFX.getClosedLoopReference());
    closedLoopReferenceSlope =
        track(MotorSignal.CLOSED_LOOP_REFERENCE_SLOPE, talonFX.getClosedLoopReferenceSlope());
    closedLoopError = track(MotorSignal.CLOSED_LOOP_ERROR, talonFX.getClosedLoopError());
    closedLoopProportionalOutput =
        track(
            MotorSignal.CLOSED_LOOP_PROPORTIONAL_OUTPUT, talonFX.getClosedLoopProportionalOutput());
    closedLoopIntegratedOutput =
        track(MotorSignal.CLOSED_LOOP_INTEGRATED_OUTPUT, talonFX.getClosedLoopIntegratedOutput());
    closedLoopDerivativeOutput =
        track(MotorSignal.CLOSED_LOOP_DERIVATIVE_OUTPUT, talonFX.getClosedLoopDerivativeOutput());
    closedLoopFeedForward =
        track(MotorSignal.CLOSED_LOOP_FEEDFORWARD, talonFX.getClosedLoopFeedForward());
    closedLoopOutput = track(MotorSignal.CLOSED_LOOP_OUTPUT, talonFX.getClosedLoopOutput());
    motionMagicAtTarget = track(MotorSignal.PROFILE_AT_TARGET, talonFX.getMotionMagicAtTarget());
    motionMagicIsRunning = track(MotorSignal.PROFILE_RUNNING, talonFX.getMotionMagicIsRunning());
    differentialControlMode =
        track(MotorSignal.DIFFERENTIAL_CONTROL_MODE, talonFX.getDifferentialControlMode());
    differentialAveragePosition =
        track(MotorSignal.DIFFERENTIAL_AVERAGE_POSITION, talonFX.getDifferentialAveragePosition());
    differentialAverageVelocity =
        track(MotorSignal.DIFFERENTIAL_AVERAGE_VELOCITY, talonFX.getDifferentialAverageVelocity());
    differentialDifferencePosition =
        track(
            MotorSignal.DIFFERENTIAL_DIFFERENCE_POSITION,
            talonFX.getDifferentialDifferencePosition());
    differentialDifferenceVelocity =
        track(
            MotorSignal.DIFFERENTIAL_DIFFERENCE_VELOCITY,
            talonFX.getDifferentialDifferenceVelocity());
    differentialClosedLoopSlot =
        track(MotorSignal.DIFFERENTIAL_GAIN_SLOT, talonFX.getDifferentialClosedLoopSlot());
    differentialOutput = track(MotorSignal.DIFFERENTIAL_OUTPUT, talonFX.getDifferentialOutput());
    differentialClosedLoopProportionalOutput =
        track(
            MotorSignal.DIFFERENTIAL_CLOSED_LOOP_PROPORTIONAL_OUTPUT,
            talonFX.getDifferentialClosedLoopProportionalOutput());
    differentialClosedLoopIntegratedOutput =
        track(
            MotorSignal.DIFFERENTIAL_CLOSED_LOOP_INTEGRATED_OUTPUT,
            talonFX.getDifferentialClosedLoopIntegratedOutput());
    differentialClosedLoopDerivativeOutput =
        track(
            MotorSignal.DIFFERENTIAL_CLOSED_LOOP_DERIVATIVE_OUTPUT,
            talonFX.getDifferentialClosedLoopDerivativeOutput());
    differentialClosedLoopFeedForward =
        track(
            MotorSignal.DIFFERENTIAL_CLOSED_LOOP_FEEDFORWARD,
            talonFX.getDifferentialClosedLoopFeedForward());
    differentialClosedLoopOutput =
        track(
            MotorSignal.DIFFERENTIAL_CLOSED_LOOP_OUTPUT, talonFX.getDifferentialClosedLoopOutput());
    differentialClosedLoopReference =
        track(
            MotorSignal.DIFFERENTIAL_CLOSED_LOOP_REFERENCE,
            talonFX.getDifferentialClosedLoopReference());
    differentialClosedLoopReferenceSlope =
        track(
            MotorSignal.DIFFERENTIAL_CLOSED_LOOP_REFERENCE_SLOPE,
            talonFX.getDifferentialClosedLoopReferenceSlope());
    differentialClosedLoopError =
        track(MotorSignal.DIFFERENTIAL_CLOSED_LOOP_ERROR, talonFX.getDifferentialClosedLoopError());
    forwardLimit = track(MotorSignal.FORWARD_LIMIT_SWITCH_ASSERTED, talonFX.getForwardLimit());
    reverseLimit = track(MotorSignal.REVERSE_LIMIT_SWITCH_ASSERTED, talonFX.getReverseLimit());
    faults =
        signals.trackFaults(
            talonFX, TalonFXFault.class, MotorSignal.FAULTS, MotorSignal.STICKY_FAULTS);
    signals.apply(talonFX, canBus);

    if (odometryThread != null) {
      odometryTimestamps = odometryThread.makeTimestampQueue();
      odometryPositionRotations = odometryThread.registerSignal(position);
    } else {
      odometryTimestamps = null;
      odometryPositionRotations = null;
    }
  }

  /**
   * Returns a factory for TalonFXs whose positions are sampled by an odometry thread.
   *
   * @param odometryThread the thread
   * @return the factory
   */
  public static MotorIOFactory factory(PhoenixOdometryThread odometryThread) {
    return new MotorIOFactory() {
      @Override
      public MotorIO create(MotorConfig config) {
        return new MotorIOTalonFX(config, odometryThread);
      }

      @Override
      public String family() {
        return FAMILY;
      }

      @Override
      public boolean supportsHardwareFollowing() {
        return true;
      }
    };
  }

  private <T> StatusSignal<T> track(MotorSignal signal, StatusSignal<T> status) {
    return signals.track(signal, status);
  }

  @Override
  @Trace
  public void updateInputs(MotorTelemetry t) {
    connected = BaseStatusSignal.isAllGood(position, velocity, motorVoltage);
    t.connected = connected;
    t.firmwareVersion = version.getValue();
    t.isProLicensed = isProLicensed.getValue();
    t.position = position.getValue();
    t.velocity = velocity.getValue();
    t.acceleration = acceleration.getValue();
    t.rotorPosition = rotorPosition.getValue();
    t.rotorVelocity = rotorVelocity.getValue();
    t.appliedVoltage = motorVoltage.getValue();
    t.supplyVoltage = supplyVoltage.getValue();
    t.dutyCycle = dutyCycle.getValue();
    t.supplyCurrent = supplyCurrent.getValue();
    t.outputCurrent = statorCurrent.getValue();
    t.torqueCurrent = torqueCurrent.getValue();
    t.deviceTemperature = deviceTemp.getValue();
    t.processorTemperature = processorTemp.getValue();
    t.ancillaryDeviceTemperature = ancillaryDeviceTemp.getValue();
    t.rotorPolarity = TalonFXConversions.toRotorPolarity(rotorPolarity.getValue());
    t.connectedMotor = TalonFXConversions.toConnectedMotor(connectedMotor.getValue());
    t.motorKT = motorKT.getValue();
    t.motorKV = motorKV.getValue();
    t.motorStallCurrent = motorStallCurrent.getValue();
    t.robotEnabled = robotEnable.getValue() == RobotEnableValue.Enabled;
    t.deviceEnabled = deviceEnable.getValue() == DeviceEnableValue.Enabled;
    t.motorOutputStatus = TalonFXConversions.toMotorOutputStatus(motorOutputStatus.getValue());
    t.bridgeOutput = TalonFXConversions.toBridgeOutput(bridgeOutput.getValue());

    ControlModeValue mode = controlMode.getValue();
    t.controlMode = TalonFXConversions.toControlMode(mode);
    t.focActive = TalonFXConversions.isFOC(mode);
    t.gainSlot = TalonFXConversions.toGainSlot(closedLoopSlot.getValue());
    boolean onPosition = TalonFXConversions.isPositionMode(t.controlMode);
    boolean onVelocity = TalonFXConversions.isVelocityMode(t.controlMode);
    double reference = closedLoopReference.getValue();
    double slope = closedLoopReferenceSlope.getValue();
    double error = closedLoopError.getValue();
    t.closedLoopPositionReference = Rotations.of(onPosition ? reference : 0.0);
    t.closedLoopPositionReferenceSlope = RotationsPerSecond.of(onPosition ? slope : 0.0);
    t.closedLoopPositionError = Rotations.of(onPosition ? error : 0.0);
    t.closedLoopVelocityReference = RotationsPerSecond.of(onVelocity ? reference : 0.0);
    t.closedLoopVelocityReferenceSlope = RotationsPerSecondPerSecond.of(onVelocity ? slope : 0.0);
    t.closedLoopVelocityError = RotationsPerSecond.of(onVelocity ? error : 0.0);
    t.closedLoopProportionalOutput = closedLoopProportionalOutput.getValue();
    t.closedLoopIntegratedOutput = closedLoopIntegratedOutput.getValue();
    t.closedLoopDerivativeOutput = closedLoopDerivativeOutput.getValue();
    t.closedLoopFeedforward = closedLoopFeedForward.getValue();
    t.closedLoopOutput = closedLoopOutput.getValue();
    t.profileAtTarget = motionMagicAtTarget.getValue();
    t.profileRunning = motionMagicIsRunning.getValue();

    DifferentialControlModeValue differentialMode = differentialControlMode.getValue();
    t.differentialControlMode = TalonFXConversions.toControlMode(differentialMode);
    t.differentialFocActive = TalonFXConversions.isFOC(differentialMode);
    t.differentialAveragePosition = differentialAveragePosition.getValue();
    t.differentialAverageVelocity = differentialAverageVelocity.getValue();
    t.differentialDifferencePosition = differentialDifferencePosition.getValue();
    t.differentialDifferenceVelocity = differentialDifferenceVelocity.getValue();
    t.differentialGainSlot = TalonFXConversions.toGainSlot(differentialClosedLoopSlot.getValue());
    t.differentialOutput = differentialOutput.getValue();
    t.differentialClosedLoopProportionalOutput =
        differentialClosedLoopProportionalOutput.getValue();
    t.differentialClosedLoopIntegratedOutput = differentialClosedLoopIntegratedOutput.getValue();
    t.differentialClosedLoopDerivativeOutput = differentialClosedLoopDerivativeOutput.getValue();
    t.differentialClosedLoopFeedforward = differentialClosedLoopFeedForward.getValue();
    t.differentialClosedLoopOutput = differentialClosedLoopOutput.getValue();
    boolean differentialOnPosition = TalonFXConversions.isPositionMode(t.differentialControlMode);
    boolean differentialOnVelocity = TalonFXConversions.isVelocityMode(t.differentialControlMode);
    double differentialReference = differentialClosedLoopReference.getValue();
    double differentialSlope = differentialClosedLoopReferenceSlope.getValue();
    double differentialError = differentialClosedLoopError.getValue();
    t.differentialClosedLoopPositionReference =
        Rotations.of(differentialOnPosition ? differentialReference : 0.0);
    t.differentialClosedLoopPositionReferenceSlope =
        RotationsPerSecond.of(differentialOnPosition ? differentialSlope : 0.0);
    t.differentialClosedLoopPositionError =
        Rotations.of(differentialOnPosition ? differentialError : 0.0);
    t.differentialClosedLoopVelocityReference =
        RotationsPerSecond.of(differentialOnVelocity ? differentialReference : 0.0);
    t.differentialClosedLoopVelocityReferenceSlope =
        RotationsPerSecondPerSecond.of(differentialOnVelocity ? differentialSlope : 0.0);
    t.differentialClosedLoopVelocityError =
        RotationsPerSecond.of(differentialOnVelocity ? differentialError : 0.0);

    t.forwardLimitSwitchAsserted = forwardLimit.getValue() == ForwardLimitValue.ClosedToGround;
    t.reverseLimitSwitchAsserted = reverseLimit.getValue() == ReverseLimitValue.ClosedToGround;
    t.faults = faults.faults();
    t.stickyFaults = faults.stickyFaults();

    if (odometryTimestamps != null) {
      int samples = Math.min(odometryTimestamps.size(), odometryPositionRotations.size());
      t.odometryTimestamps = new double[samples];
      t.odometryPositions = new Angle[samples];
      for (int i = 0; i < samples; i++) {
        t.odometryTimestamps[i] = odometryTimestamps.poll();
        t.odometryPositions[i] = Rotations.of(odometryPositionRotations.poll());
      }
    }
  }

  @Override
  public boolean isConnected() {
    return connected;
  }

  @Override
  public void setControl(MotorRequest request) {
    talonFX.setControl(TalonFXConversions.toControlRequest(request));
  }

  @Override
  public void setPosition(Angle position) {
    talonFX.setPosition(position);
  }

  @Override
  public void updateGains(SlotConfig gains, GainSlot gainSlot) {
    talonFX.getConfigurator().apply(TalonFXConversions.toSlotConfigs(gains, gainSlot));
  }

  @Override
  public void updateMotionProfile(MotionProfileConfig motionProfile) {
    talonFX.getConfigurator().apply(TalonFXConversions.toMotionMagicConfigs(motionProfile));
  }

  @Override
  public void updateCurrentLimits(CurrentLimits currentLimits) {
    talonFX.getConfigurator().apply(TalonFXConversions.toCurrentLimitsConfigs(currentLimits));
  }

  @Override
  public void clearStickyFaults() {
    talonFX.clearStickyFaults();
  }
}
