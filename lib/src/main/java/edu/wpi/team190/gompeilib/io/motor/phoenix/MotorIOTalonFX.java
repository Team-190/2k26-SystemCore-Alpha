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

  private final SignalTracker signals;

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
    signals =
        new SignalTracker(config.signalRates, MotorConfig.CONTROL_FIELDS, MotorTelemetry.class);

    PhoenixUtil.tryUntilOk(
        5, () -> talonFX.getConfigurator().apply(TalonFXConversions.toConfiguration(config), 0.25));

    version = track("firmwareVersion", talonFX.getVersion());
    isProLicensed = track("isProLicensed", talonFX.getIsProLicensed());
    position = track("position", talonFX.getPosition());
    velocity = track("velocity", talonFX.getVelocity());
    acceleration = track("acceleration", talonFX.getAcceleration());
    rotorPosition = track("rotorPosition", talonFX.getRotorPosition());
    rotorVelocity = track("rotorVelocity", talonFX.getRotorVelocity());
    motorVoltage = track("appliedVoltage", talonFX.getMotorVoltage());
    supplyVoltage = track("supplyVoltage", talonFX.getSupplyVoltage());
    dutyCycle = track("dutyCycle", talonFX.getDutyCycle());
    supplyCurrent = track("supplyCurrent", talonFX.getSupplyCurrent());
    statorCurrent = track("outputCurrent", talonFX.getStatorCurrent());
    torqueCurrent = track("torqueCurrent", talonFX.getTorqueCurrent());
    deviceTemp = track("deviceTemperature", talonFX.getDeviceTemp());
    processorTemp = track("processorTemperature", talonFX.getProcessorTemp());
    ancillaryDeviceTemp = track("ancillaryDeviceTemperature", talonFX.getAncillaryDeviceTemp());
    rotorPolarity = track("rotorPolarity", talonFX.getAppliedRotorPolarity());
    connectedMotor = track("connectedMotor", talonFX.getConnectedMotor());
    motorKT = track("motorKT", talonFX.getMotorKT());
    motorKV = track("motorKV", talonFX.getMotorKV());
    motorStallCurrent = track("motorStallCurrent", talonFX.getMotorStallCurrent());
    robotEnable = track("robotEnabled", talonFX.getRobotEnable());
    deviceEnable = track("deviceEnabled", talonFX.getDeviceEnable());
    controlMode = track("controlMode", talonFX.getControlMode());
    motorOutputStatus = track("motorOutputStatus", talonFX.getMotorOutputStatus());
    bridgeOutput = track("bridgeOutput", talonFX.getBridgeOutput());
    closedLoopSlot = track("gainSlot", talonFX.getClosedLoopSlot());
    closedLoopReference = track("closedLoopPositionReference", talonFX.getClosedLoopReference());
    closedLoopReferenceSlope =
        track("closedLoopPositionReferenceSlope", talonFX.getClosedLoopReferenceSlope());
    closedLoopError = track("closedLoopPositionError", talonFX.getClosedLoopError());
    closedLoopProportionalOutput =
        track("closedLoopProportionalOutput", talonFX.getClosedLoopProportionalOutput());
    closedLoopIntegratedOutput =
        track("closedLoopIntegratedOutput", talonFX.getClosedLoopIntegratedOutput());
    closedLoopDerivativeOutput =
        track("closedLoopDerivativeOutput", talonFX.getClosedLoopDerivativeOutput());
    closedLoopFeedForward = track("closedLoopFeedforward", talonFX.getClosedLoopFeedForward());
    closedLoopOutput = track("closedLoopOutput", talonFX.getClosedLoopOutput());
    motionMagicAtTarget = track("profileAtTarget", talonFX.getMotionMagicAtTarget());
    motionMagicIsRunning = track("profileRunning", talonFX.getMotionMagicIsRunning());
    differentialControlMode =
        track("differentialControlMode", talonFX.getDifferentialControlMode());
    differentialAveragePosition =
        track("differentialAveragePosition", talonFX.getDifferentialAveragePosition());
    differentialAverageVelocity =
        track("differentialAverageVelocity", talonFX.getDifferentialAverageVelocity());
    differentialDifferencePosition =
        track("differentialDifferencePosition", talonFX.getDifferentialDifferencePosition());
    differentialDifferenceVelocity =
        track("differentialDifferenceVelocity", talonFX.getDifferentialDifferenceVelocity());
    differentialClosedLoopSlot =
        track("differentialGainSlot", talonFX.getDifferentialClosedLoopSlot());
    differentialOutput = track("differentialOutput", talonFX.getDifferentialOutput());
    differentialClosedLoopProportionalOutput =
        track(
            "differentialClosedLoopProportionalOutput",
            talonFX.getDifferentialClosedLoopProportionalOutput());
    differentialClosedLoopIntegratedOutput =
        track(
            "differentialClosedLoopIntegratedOutput",
            talonFX.getDifferentialClosedLoopIntegratedOutput());
    differentialClosedLoopDerivativeOutput =
        track(
            "differentialClosedLoopDerivativeOutput",
            talonFX.getDifferentialClosedLoopDerivativeOutput());
    differentialClosedLoopFeedForward =
        track("differentialClosedLoopFeedforward", talonFX.getDifferentialClosedLoopFeedForward());
    differentialClosedLoopOutput =
        track("differentialClosedLoopOutput", talonFX.getDifferentialClosedLoopOutput());
    differentialClosedLoopReference =
        track(
            "differentialClosedLoopPositionReference",
            talonFX.getDifferentialClosedLoopReference());
    differentialClosedLoopReferenceSlope =
        track(
            "differentialClosedLoopPositionReferenceSlope",
            talonFX.getDifferentialClosedLoopReferenceSlope());
    differentialClosedLoopError =
        track("differentialClosedLoopPositionError", talonFX.getDifferentialClosedLoopError());
    forwardLimit = track("forwardLimitSwitchAsserted", talonFX.getForwardLimit());
    reverseLimit = track("reverseLimitSwitchAsserted", talonFX.getReverseLimit());
    faults = signals.trackFaults(talonFX, TalonFXFault.class);
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

  private <T> StatusSignal<T> track(String field, StatusSignal<T> signal) {
    return signals.track(field, signal);
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
