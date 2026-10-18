package edu.wpi.team190.gompeilib.io.gyro.phoenix;

import static org.wpilib.units.Units.Degrees;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.team190.gompeilib.io.gyro.config.GyroConfig;
import edu.wpi.team190.gompeilib.io.gyro.inputs.GyroTelemetry;
import edu.wpi.team190.gompeilib.io.gyro.interfaces.GyroIO;
import edu.wpi.team190.gompeilib.io.phoenix.CANBuses;
import edu.wpi.team190.gompeilib.io.phoenix.FaultSignals;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixOdometryThread;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixUtil;
import edu.wpi.team190.gompeilib.io.phoenix.SignalTracker;
import edu.wpi.team190.gompeilib.logging.Trace;
import java.util.Queue;
import java.util.function.Consumer;
import org.wpilib.networktables.NetworkTablesJNI;
import org.wpilib.units.measure.*;

/**
 * A Pigeon2 gyro. Logs every status signal the device publishes. Given an odometry thread, it also
 * captures yaw between main-loop cycles for odometry. Roll is about X, pitch about Y, yaw about Z.
 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final Queue<Double> odometryTimestamps;
  private final Queue<Double> odometryYawDegrees;
  private final Consumer<Long> headingUpdateTimestamp;

  private final StatusSignal<Integer> version;
  private final StatusSignal<Boolean> isProLicensed;
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<Angle> roll;
  private final StatusSignal<AngularVelocity> rollVelocity;
  private final StatusSignal<AngularVelocity> pitchVelocity;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final StatusSignal<AngularVelocity> deviceAngularVelocityX;
  private final StatusSignal<AngularVelocity> deviceAngularVelocityY;
  private final StatusSignal<AngularVelocity> deviceAngularVelocityZ;
  private final StatusSignal<LinearAcceleration> accelerationX;
  private final StatusSignal<LinearAcceleration> accelerationY;
  private final StatusSignal<LinearAcceleration> accelerationZ;
  private final StatusSignal<Double> quaternionW;
  private final StatusSignal<Double> quaternionX;
  private final StatusSignal<Double> quaternionY;
  private final StatusSignal<Double> quaternionZ;
  private final StatusSignal<Double> gravityVectorX;
  private final StatusSignal<Double> gravityVectorY;
  private final StatusSignal<Double> gravityVectorZ;
  private final StatusSignal<Angle> accumulatedRotationX;
  private final StatusSignal<Angle> accumulatedRotationY;
  private final StatusSignal<Angle> accumulatedRotationZ;
  private final StatusSignal<Double> magneticFieldX;
  private final StatusSignal<Double> magneticFieldY;
  private final StatusSignal<Double> magneticFieldZ;
  private final StatusSignal<Double> rawMagneticFieldX;
  private final StatusSignal<Double> rawMagneticFieldY;
  private final StatusSignal<Double> rawMagneticFieldZ;
  private final StatusSignal<Temperature> temperature;
  private final StatusSignal<Boolean> temperatureCompensationDisabled;
  private final StatusSignal<Boolean> noMotionEnabled;
  private final StatusSignal<Double> noMotionCount;
  private final StatusSignal<Time> upTime;
  private final StatusSignal<Voltage> supplyVoltage;
  private final FaultSignals<Pigeon2Fault> faults;

  private boolean connected = false;

  /**
   * Creates and configures a Pigeon2 without odometry sampling.
   *
   * @param config the gyro's config
   */
  public GyroIOPigeon2(GyroConfig config) {
    this(config, null, null);
  }

  /**
   * Creates and configures a Pigeon2 whose yaw is also sampled by an odometry thread.
   *
   * @param config the gyro's config
   * @param odometryThread the thread that samples yaw between cycles, or null for none
   * @param headingUpdateTimestamp called with the NetworkTables time of each update, or null
   */
  public GyroIOPigeon2(
      GyroConfig config,
      PhoenixOdometryThread odometryThread,
      Consumer<Long> headingUpdateTimestamp) {
    CANBus bus = CANBuses.toCANBus(config.bus);
    pigeon = new Pigeon2(config.canId, bus);
    PhoenixUtil.tryUntilOk(
        5, () -> pigeon.getConfigurator().apply(Pigeon2Conversions.toConfiguration(config), 0.25));
    pigeon.getConfigurator().setYaw(Degrees.of(0.0));
    this.headingUpdateTimestamp = headingUpdateTimestamp;

    SignalTracker signals =
        new SignalTracker(config.signalRates, GyroConfig.CONTROL_FIELDS, GyroTelemetry.class);
    version = signals.track("firmwareVersion", pigeon.getVersion());
    isProLicensed = signals.track("isProLicensed", pigeon.getIsProLicensed());
    yaw = signals.track("yawPosition", pigeon.getYaw());
    pitch = signals.track("pitchPosition", pigeon.getPitch());
    roll = signals.track("rollPosition", pigeon.getRoll());
    rollVelocity = signals.track("rollVelocity", pigeon.getAngularVelocityXWorld());
    pitchVelocity = signals.track("pitchVelocity", pigeon.getAngularVelocityYWorld());
    yawVelocity = signals.track("yawVelocity", pigeon.getAngularVelocityZWorld());
    deviceAngularVelocityX =
        signals.track("deviceAngularVelocityX", pigeon.getAngularVelocityXDevice());
    deviceAngularVelocityY =
        signals.track("deviceAngularVelocityY", pigeon.getAngularVelocityYDevice());
    deviceAngularVelocityZ =
        signals.track("deviceAngularVelocityZ", pigeon.getAngularVelocityZDevice());
    accelerationX = signals.track("accelerationX", pigeon.getAccelerationX());
    accelerationY = signals.track("accelerationY", pigeon.getAccelerationY());
    accelerationZ = signals.track("accelerationZ", pigeon.getAccelerationZ());
    quaternionW = signals.track("quaternionW", pigeon.getQuatW());
    quaternionX = signals.track("quaternionX", pigeon.getQuatX());
    quaternionY = signals.track("quaternionY", pigeon.getQuatY());
    quaternionZ = signals.track("quaternionZ", pigeon.getQuatZ());
    gravityVectorX = signals.track("gravityVectorX", pigeon.getGravityVectorX());
    gravityVectorY = signals.track("gravityVectorY", pigeon.getGravityVectorY());
    gravityVectorZ = signals.track("gravityVectorZ", pigeon.getGravityVectorZ());
    accumulatedRotationX = signals.track("accumulatedRotationX", pigeon.getAccumGyroX());
    accumulatedRotationY = signals.track("accumulatedRotationY", pigeon.getAccumGyroY());
    accumulatedRotationZ = signals.track("accumulatedRotationZ", pigeon.getAccumGyroZ());
    magneticFieldX = signals.track("magneticFieldX", pigeon.getMagneticFieldX());
    magneticFieldY = signals.track("magneticFieldY", pigeon.getMagneticFieldY());
    magneticFieldZ = signals.track("magneticFieldZ", pigeon.getMagneticFieldZ());
    rawMagneticFieldX = signals.track("rawMagneticFieldX", pigeon.getRawMagneticFieldX());
    rawMagneticFieldY = signals.track("rawMagneticFieldY", pigeon.getRawMagneticFieldY());
    rawMagneticFieldZ = signals.track("rawMagneticFieldZ", pigeon.getRawMagneticFieldZ());
    temperature = signals.track("temperature", pigeon.getTemperature());
    temperatureCompensationDisabled =
        signals.track(
            "temperatureCompensationDisabled", pigeon.getTemperatureCompensationDisabled());
    noMotionEnabled = signals.track("noMotionEnabled", pigeon.getNoMotionEnabled());
    noMotionCount = signals.track("noMotionCount", pigeon.getNoMotionCount());
    upTime = signals.track("upTime", pigeon.getUpTime());
    supplyVoltage = signals.track("supplyVoltage", pigeon.getSupplyVoltage());
    faults = signals.trackFaults(pigeon, Pigeon2Fault.class);
    signals.apply(pigeon, bus);

    if (odometryThread != null) {
      odometryTimestamps = odometryThread.makeTimestampQueue();
      odometryYawDegrees = odometryThread.registerSignal(yaw);
    } else {
      odometryTimestamps = null;
      odometryYawDegrees = null;
    }
  }

  @Override
  @Trace
  public void updateInputs(GyroTelemetry t) {
    connected = BaseStatusSignal.isAllGood(yaw, pitch, roll, yawVelocity);
    t.connected = connected;
    t.firmwareVersion = version.getValue();
    t.isProLicensed = isProLicensed.getValue();
    t.yawPosition = yaw.getValue();
    t.pitchPosition = pitch.getValue();
    t.rollPosition = roll.getValue();
    t.rollVelocity = rollVelocity.getValue();
    t.pitchVelocity = pitchVelocity.getValue();
    t.yawVelocity = yawVelocity.getValue();
    t.deviceAngularVelocityX = deviceAngularVelocityX.getValue();
    t.deviceAngularVelocityY = deviceAngularVelocityY.getValue();
    t.deviceAngularVelocityZ = deviceAngularVelocityZ.getValue();
    t.accelerationX = accelerationX.getValue();
    t.accelerationY = accelerationY.getValue();
    t.accelerationZ = accelerationZ.getValue();
    t.quaternionW = quaternionW.getValue();
    t.quaternionX = quaternionX.getValue();
    t.quaternionY = quaternionY.getValue();
    t.quaternionZ = quaternionZ.getValue();
    t.gravityVectorX = gravityVectorX.getValue();
    t.gravityVectorY = gravityVectorY.getValue();
    t.gravityVectorZ = gravityVectorZ.getValue();
    t.accumulatedRotationX = accumulatedRotationX.getValue();
    t.accumulatedRotationY = accumulatedRotationY.getValue();
    t.accumulatedRotationZ = accumulatedRotationZ.getValue();
    t.magneticFieldX = magneticFieldX.getValue();
    t.magneticFieldY = magneticFieldY.getValue();
    t.magneticFieldZ = magneticFieldZ.getValue();
    t.rawMagneticFieldX = rawMagneticFieldX.getValue();
    t.rawMagneticFieldY = rawMagneticFieldY.getValue();
    t.rawMagneticFieldZ = rawMagneticFieldZ.getValue();
    t.temperature = temperature.getValue();
    t.temperatureCompensationDisabled = temperatureCompensationDisabled.getValue();
    t.noMotionEnabled = noMotionEnabled.getValue();
    t.noMotionCount = noMotionCount.getValue();
    t.upTime = upTime.getValue();
    t.supplyVoltage = supplyVoltage.getValue();
    t.faults = faults.faults();
    t.stickyFaults = faults.stickyFaults();

    if (odometryTimestamps != null) {
      int samples = Math.min(odometryTimestamps.size(), odometryYawDegrees.size());
      t.odometryTimestamps = new double[samples];
      t.odometryYawPositions = new Angle[samples];
      for (int i = 0; i < samples; i++) {
        t.odometryTimestamps[i] = odometryTimestamps.poll();
        t.odometryYawPositions[i] = Degrees.of(odometryYawDegrees.poll());
      }
    }
    if (headingUpdateTimestamp != null) {
      headingUpdateTimestamp.accept(NetworkTablesJNI.now());
    }
  }

  @Override
  public boolean isConnected() {
    return connected;
  }

  @Override
  public void setYaw(Angle yaw) {
    pigeon.setYaw(yaw);
  }

  @Override
  public void clearStickyFaults() {
    pigeon.clearStickyFaults();
  }
}
