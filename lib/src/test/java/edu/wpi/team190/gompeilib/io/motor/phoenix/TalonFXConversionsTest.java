package edu.wpi.team190.gompeilib.io.motor.phoenix;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyControl;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
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
import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.TestGains;
import edu.wpi.team190.gompeilib.io.IOTestSupport;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.ClosedLoopRamps;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.FeedbackSensor;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.GravityType;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorAlignment;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorDirection;
import edu.wpi.team190.gompeilib.io.motor.config.NeutralMode;
import edu.wpi.team190.gompeilib.io.motor.config.OpenLoopRamps;
import edu.wpi.team190.gompeilib.io.motor.config.RemoteSensorMode;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.io.motor.config.SoftLimits;
import edu.wpi.team190.gompeilib.io.motor.config.TorqueCurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.inputs.BridgeOutput;
import edu.wpi.team190.gompeilib.io.motor.inputs.ConnectedMotor;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorControlMode;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorOutputStatus;
import edu.wpi.team190.gompeilib.io.motor.inputs.RotorPolarity;
import edu.wpi.team190.gompeilib.io.motor.outputs.AxisRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.Differential;
import edu.wpi.team190.gompeilib.io.motor.outputs.FollowerRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequestTest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.PositionRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.motor.outputs.VelocityRequest;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismConversion;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.wpilib.units.MultUnit;
import org.wpilib.units.measure.Current;

public class TalonFXConversionsTest extends GompeiLibTestBase {

  @Test
  public void testToConfiguration() {
    var slot0 = TestGains.position(1, 2, 3, 4, 5, 6, 7);
    var slot1 = TestGains.positionKP(11);
    var slot2 = TestGains.positionKP(21);
    MotorConfig config =
        MotorConfig.builder()
            .withCanId(1)
            .withBus(CANBusId.named("rio"))
            .withDirection(MotorDirection.CLOCKWISE_POSITIVE)
            .withNeutralMode(NeutralMode.COAST)
            .withCurrentLimits(
                CurrentLimits.fromDoubles()
                    .withSupplyCurrentLimit(30)
                    .withStatorCurrentLimit(60)
                    .build())
            .withGearRatio(12.0)
            .withOpenLoopRamps(new OpenLoopRamps(Seconds.of(0.1), Seconds.of(0.2), Seconds.of(0.3)))
            .withMotionProfile(
                new MotionProfileConfig(
                    RotationsPerSecond.of(1.0),
                    RotationsPerSecondPerSecond.of(2.0),
                    RotationsPerSecondPerSecond.per(Second).of(3.0),
                    Volts.per(RotationsPerSecond).ofNative(0.12),
                    Volts.per(RotationsPerSecondPerSecond).ofNative(0.01)))
            .withSlot(GainSlot.ZERO, SlotConfig.position(slot0, GravityType.COSINE))
            .withSlot(GainSlot.ONE, SlotConfig.position(slot1))
            .withSlot(GainSlot.TWO, SlotConfig.position(slot2))
            .build();

    TalonFXConfiguration talon = TalonFXConversions.toConfiguration(config);

    assertEquals(InvertedValue.Clockwise_Positive, talon.MotorOutput.Inverted);
    assertEquals(NeutralModeValue.Coast, talon.MotorOutput.NeutralMode);
    assertEquals(30.0, talon.CurrentLimits.SupplyCurrentLimit, 1e-9);
    assertTrue(talon.CurrentLimits.SupplyCurrentLimitEnable);
    assertEquals(60.0, talon.CurrentLimits.StatorCurrentLimit, 1e-9);
    assertTrue(talon.CurrentLimits.StatorCurrentLimitEnable);
    assertEquals(12.0, talon.Feedback.SensorToMechanismRatio, 1e-9);
    assertEquals(0.1, talon.OpenLoopRamps.DutyCycleOpenLoopRampPeriod, 1e-9);
    assertEquals(0.2, talon.OpenLoopRamps.VoltageOpenLoopRampPeriod, 1e-9);
    assertEquals(0.3, talon.OpenLoopRamps.TorqueOpenLoopRampPeriod, 1e-9);
    assertEquals(1.0, talon.MotionMagic.MotionMagicCruiseVelocity, 1e-9);
    assertEquals(2.0, talon.MotionMagic.MotionMagicAcceleration, 1e-9);
    assertEquals(3.0, talon.MotionMagic.MotionMagicJerk, 1e-9);
    assertEquals(0.12, talon.MotionMagic.MotionMagicExpo_kV, 1e-9);
    assertEquals(0.01, talon.MotionMagic.MotionMagicExpo_kA, 1e-9);
    assertEquals(1.0, talon.Slot0.kP);
    assertEquals(2.0, talon.Slot0.kI);
    assertEquals(3.0, talon.Slot0.kD);
    assertEquals(4.0, talon.Slot0.kS);
    assertEquals(5.0, talon.Slot0.kV);
    assertEquals(6.0, talon.Slot0.kA);
    assertEquals(7.0, talon.Slot0.kG);
    assertEquals(GravityTypeValue.Arm_Cosine, talon.Slot0.GravityType);
    assertEquals(11.0, talon.Slot1.kP);
    assertEquals(GravityTypeValue.Elevator_Static, talon.Slot1.GravityType);
    assertEquals(21.0, talon.Slot2.kP);
  }

  @Test
  public void testDefaultConfigurationIsCounterClockwiseBrake() {
    MotorConfig config =
        MotorConfig.builder()
            .withCanId(1)
            .withBus(CANBusId.named("rio"))
            .withCurrentLimits(
                CurrentLimits.fromDoubles()
                    .withSupplyCurrentLimit(30)
                    .withStatorCurrentLimit(60)
                    .build())
            .build();

    TalonFXConfiguration talon = TalonFXConversions.toConfiguration(config);

    assertEquals(InvertedValue.CounterClockwise_Positive, talon.MotorOutput.Inverted);
    assertEquals(NeutralModeValue.Brake, talon.MotorOutput.NeutralMode);
  }

  @Test
  public void testMapsLimitsFeedbackAndRamps() {
    MotorConfig.MotorConfigBuilder builder =
        MotorConfig.builder()
            .withCanId(1)
            .withBus(CANBusId.named("rio"))
            .withCurrentLimits(new CurrentLimits((Current) null, null))
            .withTorqueCurrentLimits(new TorqueCurrentLimits(Amps.of(60.0), Amps.of(0.0)))
            .withSoftLimits(SoftLimits.between(Rotations.of(-1.0), Rotations.of(2.0)))
            .withGearRatio(12.0)
            .withFeedbackSensor(new FeedbackSensor.RemoteEncoder(9, RemoteSensorMode.FUSED))
            .withContinuousWrap(true)
            .withClosedLoopRamps(
                new ClosedLoopRamps(Seconds.of(0.1), Seconds.of(0.2), Seconds.of(0.3)));

    TalonFXConfiguration talon = TalonFXConversions.toConfiguration(builder.build());

    assertFalse(talon.CurrentLimits.SupplyCurrentLimitEnable);
    assertFalse(talon.CurrentLimits.StatorCurrentLimitEnable);
    assertEquals(60.0, talon.TorqueCurrent.PeakForwardTorqueCurrent);
    assertEquals(0.0, talon.TorqueCurrent.PeakReverseTorqueCurrent);
    assertTrue(talon.SoftwareLimitSwitch.ForwardSoftLimitEnable);
    assertEquals(2.0, talon.SoftwareLimitSwitch.ForwardSoftLimitThreshold);
    assertTrue(talon.SoftwareLimitSwitch.ReverseSoftLimitEnable);
    assertEquals(-1.0, talon.SoftwareLimitSwitch.ReverseSoftLimitThreshold);
    assertEquals(9, talon.Feedback.FeedbackRemoteSensorID);
    assertEquals(FeedbackSensorSourceValue.FusedCANcoder, talon.Feedback.FeedbackSensorSource);
    assertEquals(12.0, talon.Feedback.RotorToSensorRatio);
    assertEquals(1.0, talon.Feedback.SensorToMechanismRatio);
    assertTrue(talon.ClosedLoopGeneral.ContinuousWrap);
    assertEquals(0.1, talon.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod);
    assertEquals(0.2, talon.ClosedLoopRamps.VoltageClosedLoopRampPeriod);
    assertEquals(0.3, talon.ClosedLoopRamps.TorqueClosedLoopRampPeriod);

    assertEquals(
        FeedbackSensorSourceValue.RemoteCANcoder,
        TalonFXConversions.toConfiguration(
                builder
                    .withFeedbackSensor(
                        new FeedbackSensor.RemoteEncoder(9, RemoteSensorMode.REMOTE))
                    .build())
            .Feedback
            .FeedbackSensorSource);
    assertEquals(
        FeedbackSensorSourceValue.SyncCANcoder,
        TalonFXConversions.toConfiguration(
                builder
                    .withFeedbackSensor(new FeedbackSensor.RemoteEncoder(9, RemoteSensorMode.SYNC))
                    .build())
            .Feedback
            .FeedbackSensorSource);
  }

  @Test
  public void testToMotionMagicConfigs() {
    MotionMagicConfigs configs =
        TalonFXConversions.toMotionMagicConfigs(
            MotionProfileConfig.trapezoidal(
                RotationsPerSecond.of(3.0), RotationsPerSecondPerSecond.of(6.0)));

    assertEquals(3.0, configs.MotionMagicCruiseVelocity);
    assertEquals(6.0, configs.MotionMagicAcceleration);
    assertEquals(0.0, configs.MotionMagicJerk);
  }

  @Test
  public void testToSlotConfigs() {
    SlotConfigs slot =
        TalonFXConversions.toSlotConfigs(
            SlotConfig.position(TestGains.position(1, 0, 0, 0, 0, 0, 2)), GainSlot.TWO);

    assertEquals(2, slot.SlotNumber);
    assertEquals(1.0, slot.kP);
    assertEquals(2.0, slot.kG);
    assertEquals(GravityTypeValue.Elevator_Static, slot.GravityType);
  }

  @Test
  public void shouldWritePositionGainsPerRotation() {
    // Declared per radian, as a simulation would tune them
    SlotConfigs slot =
        TalonFXConversions.toSlotConfigs(
            SlotConfig.position(
                Gains.angularPosition(Volts)
                    .withKP(Volts.per(Radians).ofNative(1))
                    .withKI(Volts.per(MultUnit.combine(Radians, Seconds)).ofNative(2))
                    .withKD(Volts.per(RadiansPerSecond).ofNative(3))
                    .withKS(Millivolts.of(400))
                    .withKV(Volts.per(RadiansPerSecond).ofNative(5))
                    .withKA(Volts.per(RadiansPerSecondPerSecond).ofNative(6))
                    .withKG(Volts.of(7))
                    .build(),
                GravityType.COSINE),
            GainSlot.ZERO);

    double radiansPerRotation = 2 * Math.PI;
    assertEquals(1 * radiansPerRotation, slot.kP, 1e-9);
    assertEquals(2 * radiansPerRotation, slot.kI, 1e-9);
    assertEquals(3 * radiansPerRotation, slot.kD, 1e-9);
    assertEquals(0.4, slot.kS, 1e-9);
    assertEquals(5 * radiansPerRotation, slot.kV, 1e-9);
    assertEquals(6 * radiansPerRotation, slot.kA, 1e-9);
    assertEquals(7, slot.kG, 1e-9);
    assertEquals(GravityTypeValue.Arm_Cosine, slot.GravityType);
  }

  @Test
  public void shouldWriteVelocityGainsPerRotationPerSecond() {
    SlotConfigs slot =
        TalonFXConversions.toSlotConfigs(
            SlotConfig.velocity(
                Gains.angularVelocity(Amps)
                    .withKP(Amps.per(RadiansPerSecond).ofNative(1))
                    .withKI(Amps.per(MultUnit.combine(RadiansPerSecond, Seconds)).ofNative(2))
                    .withKD(Amps.per(RadiansPerSecondPerSecond).ofNative(3))
                    .withKS(Amps.of(4))
                    .withKV(Amps.per(RadiansPerSecond).ofNative(5))
                    .withKA(Amps.per(RadiansPerSecondPerSecond).ofNative(6))
                    .build()),
            GainSlot.ONE);

    double radiansPerRotation = 2 * Math.PI;
    assertEquals(1, slot.SlotNumber);
    assertEquals(1 * radiansPerRotation, slot.kP, 1e-9);
    assertEquals(2 * radiansPerRotation, slot.kI, 1e-9);
    assertEquals(3 * radiansPerRotation, slot.kD, 1e-9);
    assertEquals(4, slot.kS, 1e-9);
    assertEquals(5 * radiansPerRotation, slot.kV, 1e-9);
    assertEquals(6 * radiansPerRotation, slot.kA, 1e-9);
    assertEquals(0, slot.kG, 1e-9);
  }

  @Test
  public void shouldWriteDutyCycleGainsAsFractions() {
    SlotConfigs slot =
        TalonFXConversions.toSlotConfigs(
            SlotConfig.position(
                Gains.angularPosition(Percent)
                    .withKP(Percent.per(Rotations).ofNative(50))
                    .withKS(Percent.of(5))
                    .build()),
            GainSlot.ZERO);

    assertEquals(0.5, slot.kP, 1e-9);
    assertEquals(0.05, slot.kS, 1e-9);
  }

  @Test
  public void shouldWriteLinearGainsPerDrumRotation() {
    // 10 V/m through a drum of radius 0.05 m: one rotation moves 2 pi 0.05 m
    var perMeter =
        Gains.linearPosition(Volts)
            .withKP(Volts.per(Meters).ofNative(10))
            .withKV(Volts.per(MetersPerSecond).ofNative(4))
            .build();
    SlotConfigs slot =
        TalonFXConversions.toSlotConfigs(
            SlotConfig.position(
                MechanismConversion.drum(Meters.of(0.05)).toMotorPositionGains(perMeter)),
            GainSlot.ZERO);

    double metersPerRotation = 2 * Math.PI * 0.05;
    assertEquals(10 * metersPerRotation, slot.kP, 1e-9);
    assertEquals(4 * metersPerRotation, slot.kV, 1e-9);
  }

  @Test
  public void testEverySingleRequestMapsToItsPhoenixRequest() throws ReflectiveOperationException {
    Map<Class<?>, String> phoenixNames = new HashMap<>();
    MotorRequestTest.TALONFX_SINGLE_REQUESTS.forEach(
        (phoenix, ours) -> phoenixNames.put(ours, phoenix));

    for (Class<?> type : IOTestSupport.leafRecords(MotorRequest.class)) {
      if (DIFFERENTIAL_REQUESTS.contains(type)) {
        continue;
      }
      MotorRequest request = (MotorRequest) IOTestSupport.sampleRecord(type);
      ControlRequest phoenix = TalonFXConversions.toControlRequest(request);
      String name = type.getSimpleName();

      assertEquals(phoenixNames.get(type), phoenix.getClass().getSimpleName(), name);
      if (request instanceof AxisRequest axis) {
        assertOptionsApplied(phoenix, axis.options(), name);
      }
    }
  }

  private static void assertOptionsApplied(
      ControlRequest phoenix, RequestOptions options, String name)
      throws ReflectiveOperationException {
    Class<?> type = phoenix.getClass();
    String override =
        hasField(type, "OverrideCoastDurNeutral")
            ? "OverrideCoastDurNeutral"
            : "OverrideBrakeDurNeutral";
    assertEquals(options.overrideNeutralMode(), type.getField(override).get(phoenix), name);
    assertEquals(
        options.limitForwardMotion(), type.getField("LimitForwardMotion").get(phoenix), name);
    assertEquals(
        options.limitReverseMotion(), type.getField("LimitReverseMotion").get(phoenix), name);
    assertEquals(
        options.ignoreHardwareLimits(), type.getField("IgnoreHardwareLimits").get(phoenix), name);
    assertEquals(
        options.ignoreSoftwareLimits(), type.getField("IgnoreSoftwareLimits").get(phoenix), name);
    assertEquals(options.useTimesync(), type.getField("UseTimesync").get(phoenix), name);
    assertEquals(
        options.updateFrequency().in(Hertz),
        (double) type.getField("UpdateFreqHz").get(phoenix),
        1e-9,
        name);
  }

  private static boolean hasField(Class<?> type, String name) {
    for (Field field : type.getFields()) {
      if (field.getName().equals(name)) {
        return true;
      }
    }
    return false;
  }

  @Test
  public void testRequestTargetsAreConverted() throws ReflectiveOperationException {
    ControlRequest phoenix =
        TalonFXConversions.toControlRequest(
            new edu.wpi.team190.gompeilib.io.motor.outputs.PositionRequest.PositionVoltage(
                Rotations.of(2.5),
                RotationsPerSecond.of(1.5),
                Volts.of(0.75),
                false,
                GainSlot.TWO,
                RequestOptions.DEFAULT));

    assertEquals(2.5, (double) phoenix.getClass().getField("Position").get(phoenix), 1e-9);
    assertEquals(1.5, (double) phoenix.getClass().getField("Velocity").get(phoenix), 1e-9);
    assertEquals(0.75, (double) phoenix.getClass().getField("FeedForward").get(phoenix), 1e-9);
    assertEquals(false, phoenix.getClass().getField("EnableFOC").get(phoenix));
    assertEquals(2, phoenix.getClass().getField("Slot").get(phoenix));
  }

  private static final Set<Class<?>> DIFFERENTIAL_REQUESTS =
      Set.of(
          Differential.class,
          FollowerRequest.DifferentialFollow.class,
          FollowerRequest.DifferentialStrictFollow.class);

  @Test
  public void testConvertsDifferentialRequests() {
    ControlRequest follow =
        TalonFXConversions.toControlRequest(
            new FollowerRequest.DifferentialFollow(4, MotorAlignment.OPPOSED));
    assertInstanceOf(com.ctre.phoenix6.controls.DifferentialFollower.class, follow);
    com.ctre.phoenix6.controls.DifferentialFollower diffFollower =
        (com.ctre.phoenix6.controls.DifferentialFollower) follow;
    assertEquals(4, diffFollower.LeaderID);
    assertEquals(MotorAlignmentValue.Opposed, diffFollower.MotorAlignment);

    ControlRequest strictFollow =
        TalonFXConversions.toControlRequest(new FollowerRequest.DifferentialStrictFollow(5));
    assertInstanceOf(com.ctre.phoenix6.controls.DifferentialStrictFollower.class, strictFollow);
    com.ctre.phoenix6.controls.DifferentialStrictFollower diffStrictFollower =
        (com.ctre.phoenix6.controls.DifferentialStrictFollower) strictFollow;
    assertEquals(5, diffStrictFollower.LeaderID);

    Differential diff =
        new Differential(
            new PositionRequest.PositionVoltage(
                Rotations.of(1.0),
                RotationsPerSecond.of(0.0),
                Volts.of(0.0),
                false,
                GainSlot.ZERO,
                RequestOptions.DEFAULT),
            new PositionRequest.PositionVoltage(
                Rotations.of(0.5),
                RotationsPerSecond.of(0.0),
                Volts.of(0.0),
                false,
                GainSlot.ONE,
                RequestOptions.DEFAULT));
    ControlRequest compound = TalonFXConversions.toControlRequest(diff);
    assertInstanceOf(
        com.ctre.phoenix6.controls.compound.Diff_PositionVoltage_Position.class, compound);

    Differential diffOpen =
        new Differential(
            new PositionRequest.PositionVoltage(
                Rotations.of(1.0),
                RotationsPerSecond.of(0.0),
                Volts.of(0.0),
                false,
                GainSlot.ZERO,
                RequestOptions.DEFAULT),
            new OpenLoopRequest.VoltageOutput(Volts.of(2.0)));
    ControlRequest compoundOpen = TalonFXConversions.toControlRequest(diffOpen);
    assertInstanceOf(
        com.ctre.phoenix6.controls.compound.Diff_PositionVoltage_Open.class, compoundOpen);

    Differential diffVel =
        new Differential(
            new PositionRequest.PositionVoltage(
                Rotations.of(1.0),
                RotationsPerSecond.of(0.0),
                Volts.of(0.0),
                false,
                GainSlot.ZERO,
                RequestOptions.DEFAULT),
            new VelocityRequest.VelocityVoltage(
                RotationsPerSecond.of(5.0),
                RotationsPerSecondPerSecond.of(0.0),
                Volts.of(0.0),
                false,
                GainSlot.TWO,
                RequestOptions.DEFAULT));
    ControlRequest compoundVel = TalonFXConversions.toControlRequest(diffVel);
    assertInstanceOf(
        com.ctre.phoenix6.controls.compound.Diff_PositionVoltage_Velocity.class, compoundVel);
  }

  @Test
  public void shouldWrapReflectiveErrorInDifferentialConversion() {
    assertThrows(
        IllegalArgumentException.class,
        () ->
            TalonFXConversions.toDifferentialCompound(
                "Invalid",
                "Open",
                new Object(),
                new Object(),
                RequestOptions.DEFAULT_UPDATE_FREQUENCY));
  }

  /** Phoenix6 request field name to {@code MotorRequest} record component name. */
  private static final Map<String, String> MOTOR_FIELD_NAMES =
      Map.ofEntries(
          Map.entry("Output", "output"),
          Map.entry("Position", "position"),
          Map.entry("Velocity", "velocity"),
          Map.entry("Acceleration", "acceleration"),
          Map.entry("Jerk", "jerk"),
          Map.entry("kV", "kV"),
          Map.entry("kA", "kA"),
          Map.entry("FeedForward", "feedforward"),
          Map.entry("EnableFOC", "foc"),
          Map.entry("Slot", "slot"),
          Map.entry("MaxAbsDutyCycle", "maxAbsDutyCycle"),
          Map.entry("Deadband", "deadband"),
          Map.entry("LeaderID", "leaderId"),
          Map.entry("MotorAlignment", "alignment"),
          Map.entry("AudioFrequency", "audioFrequency"),
          Map.entry("UseTimesync", "useTimesync"),
          Map.entry("UpdateFreqHz", "updateFrequency"),
          Map.entry("AverageRequest", "average"),
          Map.entry("DifferentialRequest", "difference"));

  /** Phoenix6 request fields a {@code RequestOptions} component carries. */
  private static final Set<String> MOTOR_OPTION_FIELDS =
      Set.of(
          "OverrideBrakeDurNeutral",
          "OverrideCoastDurNeutral",
          "LimitForwardMotion",
          "LimitReverseMotion",
          "IgnoreHardwareLimits",
          "IgnoreSoftwareLimits",
          "UseTimesync",
          "UpdateFreqHz");

  @Test
  public void testEverySingleRequestCarriesEveryPhoenixSetting() throws ClassNotFoundException {
    for (Map.Entry<String, Class<? extends MotorRequest>> mapping :
        MotorRequestTest.TALONFX_SINGLE_REQUESTS.entrySet()) {
      IOTestSupport.assertFieldsCovered(
          Class.forName("com.ctre.phoenix6.controls." + mapping.getKey()),
          mapping.getValue(),
          MOTOR_FIELD_NAMES,
          MOTOR_OPTION_FIELDS);
    }
  }

  @Test
  public void testDifferentialCarriesEveryCompoundSetting() {
    int compounds = 0;
    for (Class<?> phoenix :
        IOTestSupport.controlRequests(
            CoreTalonFX.class, Set.of(ControlRequest.class, EmptyControl.class))) {
      if (phoenix.getSimpleName().startsWith("Diff_")) {
        compounds++;
        IOTestSupport.assertFieldsCovered(phoenix, Differential.class, MOTOR_FIELD_NAMES, Set.of());
      }
    }
    assertEquals(54, compounds);
  }

  @Test
  public void shouldRejectRequestWithoutOptions() {
    assertThrows(
        IllegalStateException.class,
        () -> TalonFXConversions.options(new Object(), RequestOptions.DEFAULT));
  }

  @Test
  public void testToAlignment() {
    assertEquals(
        MotorAlignmentValue.Aligned, TalonFXConversions.toAlignment(MotorAlignment.ALIGNED));
    assertEquals(
        MotorAlignmentValue.Opposed, TalonFXConversions.toAlignment(MotorAlignment.OPPOSED));
  }

  @Test
  public void testEveryControlModeMaps() {
    for (ControlModeValue mode : ControlModeValue.values()) {
      MotorControlMode converted = TalonFXConversions.toControlMode(mode);
      if (mode == ControlModeValue.Reserved) {
        assertEquals(MotorControlMode.UNKNOWN, converted);
      } else {
        assertNotEquals(MotorControlMode.UNKNOWN, converted, mode.name());
      }
      assertEquals(mode.name().endsWith("FOC"), TalonFXConversions.isFOC(mode), mode.name());
    }
    assertEquals(
        MotorControlMode.POSITION_VOLTAGE,
        TalonFXConversions.toControlMode(ControlModeValue.PositionVoltageFOC));
    assertEquals(
        MotorControlMode.EXPONENTIAL_PROFILED_POSITION_CURRENT,
        TalonFXConversions.toControlMode(ControlModeValue.MotionMagicExpoTorqueCurrentFOC));
    assertEquals(
        MotorControlMode.PROFILED_VELOCITY_DUTY_CYCLE,
        TalonFXConversions.toControlMode(ControlModeValue.MotionMagicVelocityDutyCycle));
  }

  @Test
  public void testEveryDifferentialControlModeMaps() {
    for (DifferentialControlModeValue mode : DifferentialControlModeValue.values()) {
      ControlModeValue same = ControlModeValue.valueOf(mode.name());
      assertEquals(TalonFXConversions.toControlMode(same), TalonFXConversions.toControlMode(mode));
      assertEquals(TalonFXConversions.isFOC(same), TalonFXConversions.isFOC(mode));
    }
  }

  @Test
  public void testLoopFamilies() {
    assertTrue(TalonFXConversions.isPositionMode(MotorControlMode.POSITION_CURRENT));
    assertTrue(
        TalonFXConversions.isPositionMode(MotorControlMode.EXPONENTIAL_PROFILED_POSITION_VOLTAGE));
    assertFalse(TalonFXConversions.isPositionMode(MotorControlMode.VELOCITY_VOLTAGE));
    assertTrue(TalonFXConversions.isVelocityMode(MotorControlMode.PROFILED_VELOCITY_CURRENT));
    assertFalse(TalonFXConversions.isVelocityMode(MotorControlMode.VOLTAGE_OUTPUT));
    assertFalse(TalonFXConversions.isPositionMode(MotorControlMode.FOLLOWER));
  }

  @Test
  public void testEnumConversions() {
    for (AppliedRotorPolarityValue value : AppliedRotorPolarityValue.values()) {
      assertNotEquals(RotorPolarity.UNKNOWN, TalonFXConversions.toRotorPolarity(value));
    }
    assertEquals(
        RotorPolarity.CLOCKWISE_POSITIVE,
        TalonFXConversions.toRotorPolarity(AppliedRotorPolarityValue.PositiveIsClockwise));
    for (MotorOutputStatusValue value : MotorOutputStatusValue.values()) {
      assertEquals(
          value.name().replaceAll("([a-z])([A-Z])", "$1_$2").toUpperCase(),
          TalonFXConversions.toMotorOutputStatus(value).name());
    }
    for (BridgeOutputValue value : BridgeOutputValue.values()) {
      assertNotEquals(BridgeOutput.UNKNOWN, TalonFXConversions.toBridgeOutput(value), value.name());
    }
    assertEquals(
        BridgeOutput.TRAPEZOIDAL,
        TalonFXConversions.toBridgeOutput(BridgeOutputValue.BridgeReq_Trapez));
    assertEquals(
        BridgeOutput.FOC_TORQUE,
        TalonFXConversions.toBridgeOutput(BridgeOutputValue.BridgeReq_FOCTorque));
    for (ConnectedMotorValue value : ConnectedMotorValue.values()) {
      ConnectedMotor converted = TalonFXConversions.toConnectedMotor(value);
      assertEquals(
          value == ConnectedMotorValue.Unknown, converted == ConnectedMotor.UNKNOWN, value.name());
    }
    assertEquals(
        ConnectedMotor.KRAKEN_X60,
        TalonFXConversions.toConnectedMotor(ConnectedMotorValue.KrakenX60_Integrated));
    assertEquals(
        MotorOutputStatus.DISCORDANT_MOTORING,
        TalonFXConversions.toMotorOutputStatus(MotorOutputStatusValue.DiscordantMotoring));
  }

  @Test
  public void testToGainSlot() {
    assertEquals(GainSlot.ZERO, TalonFXConversions.toGainSlot(0));
    assertEquals(GainSlot.ONE, TalonFXConversions.toGainSlot(1));
    assertEquals(GainSlot.TWO, TalonFXConversions.toGainSlot(2));
    assertEquals(GainSlot.ZERO, TalonFXConversions.toGainSlot(7));
  }
}
