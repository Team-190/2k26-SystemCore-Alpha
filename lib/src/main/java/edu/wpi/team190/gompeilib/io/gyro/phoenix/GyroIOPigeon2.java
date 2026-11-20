package edu.wpi.team190.gompeilib.io.gyro.phoenix;

import static org.wpilib.units.Units.Degrees;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.team190.gompeilib.io.gyro.config.GyroConfig;
import edu.wpi.team190.gompeilib.io.gyro.config.GyroSignal;
import edu.wpi.team190.gompeilib.io.gyro.config.GyroSignalGroup;
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

    SignalTracker<GyroSignal, GyroSignalGroup> signals = new SignalTracker<>(config.signalRates);
    version = signals.track(GyroSignal.FIRMWARE_VERSION, pigeon.getVersion());
    isProLicensed = signals.track(GyroSignal.IS_PRO_LICENSED, pigeon.getIsProLicensed());
    yaw = signals.track(GyroSignal.YAW_POSITION, pigeon.getYaw());
    pitch = signals.track(GyroSignal.PITCH_POSITION, pigeon.getPitch());
    roll = signals.track(GyroSignal.ROLL_POSITION, pigeon.getRoll());
    rollVelocity = signals.track(GyroSignal.ROLL_VELOCITY, pigeon.getAngularVelocityXWorld());
    pitchVelocity = signals.track(GyroSignal.PITCH_VELOCITY, pigeon.getAngularVelocityYWorld());
    yawVelocity = signals.track(GyroSignal.YAW_VELOCITY, pigeon.getAngularVelocityZWorld());
    deviceAngularVelocityX =
        signals.track(GyroSignal.DEVICE_ANGULAR_VELOCITY_X, pigeon.getAngularVelocityXDevice());
    deviceAngularVelocityY =
        signals.track(GyroSignal.DEVICE_ANGULAR_VELOCITY_Y, pigeon.getAngularVelocityYDevice());
    deviceAngularVelocityZ =
        signals.track(GyroSignal.DEVICE_ANGULAR_VELOCITY_Z, pigeon.getAngularVelocityZDevice());
    accelerationX = signals.track(GyroSignal.ACCELERATION_X, pigeon.getAccelerationX());
    accelerationY = signals.track(GyroSignal.ACCELERATION_Y, pigeon.getAccelerationY());
    accelerationZ = signals.track(GyroSignal.ACCELERATION_Z, pigeon.getAccelerationZ());
    quaternionW = signals.track(GyroSignal.QUATERNION_W, pigeon.getQuatW());
    quaternionX = signals.track(GyroSignal.QUATERNION_X, pigeon.getQuatX());
    quaternionY = signals.track(GyroSignal.QUATERNION_Y, pigeon.getQuatY());
    quaternionZ = signals.track(GyroSignal.QUATERNION_Z, pigeon.getQuatZ());
    gravityVectorX = signals.track(GyroSignal.GRAVITY_VECTOR_X, pigeon.getGravityVectorX());
    gravityVectorY = signals.track(GyroSignal.GRAVITY_VECTOR_Y, pigeon.getGravityVectorY());
    gravityVectorZ = signals.track(GyroSignal.GRAVITY_VECTOR_Z, pigeon.getGravityVectorZ());
    accumulatedRotationX = signals.track(GyroSignal.ACCUMULATED_ROTATION_X, pigeon.getAccumGyroX());
    accumulatedRotationY = signals.track(GyroSignal.ACCUMULATED_ROTATION_Y, pigeon.getAccumGyroY());
    accumulatedRotationZ = signals.track(GyroSignal.ACCUMULATED_ROTATION_Z, pigeon.getAccumGyroZ());
    magneticFieldX = signals.track(GyroSignal.MAGNETIC_FIELD_X, pigeon.getMagneticFieldX());
    magneticFieldY = signals.track(GyroSignal.MAGNETIC_FIELD_Y, pigeon.getMagneticFieldY());
    magneticFieldZ = signals.track(GyroSignal.MAGNETIC_FIELD_Z, pigeon.getMagneticFieldZ());
    rawMagneticFieldX =
        signals.track(GyroSignal.RAW_MAGNETIC_FIELD_X, pigeon.getRawMagneticFieldX());
    rawMagneticFieldY =
        signals.track(GyroSignal.RAW_MAGNETIC_FIELD_Y, pigeon.getRawMagneticFieldY());
    rawMagneticFieldZ =
        signals.track(GyroSignal.RAW_MAGNETIC_FIELD_Z, pigeon.getRawMagneticFieldZ());
    temperature = signals.track(GyroSignal.TEMPERATURE, pigeon.getTemperature());
    temperatureCompensationDisabled =
        signals.track(
            GyroSignal.TEMPERATURE_COMPENSATION_DISABLED,
            pigeon.getTemperatureCompensationDisabled());
    noMotionEnabled = signals.track(GyroSignal.NO_MOTION_ENABLED, pigeon.getNoMotionEnabled());
    noMotionCount = signals.track(GyroSignal.NO_MOTION_COUNT, pigeon.getNoMotionCount());
    upTime = signals.track(GyroSignal.UP_TIME, pigeon.getUpTime());
    supplyVoltage = signals.track(GyroSignal.SUPPLY_VOLTAGE, pigeon.getSupplyVoltage());
    faults =
        signals.trackFaults(
            pigeon, Pigeon2Fault.class, GyroSignal.FAULTS, GyroSignal.STICKY_FAULTS);
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
