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
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.wpilib.units.measure.Current;

public class TalonFXConversionsTest extends GompeiLibTestBase {

  @Test
  public void testToConfiguration() {
    Gains slot0 =
        Gains.builder()
            .withKP(1)
            .withKI(2)
            .withKD(3)
            .withKS(4)
            .withKV(5)
            .withKA(6)
            .withKG(7)
            .build();
    Gains slot1 = Gains.builder().withKP(11).build();
    Gains slot2 = Gains.builder().withKP(21).build();
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
            .withSlot(GainSlot.ZERO, new SlotConfig(slot0, GravityType.COSINE))
            .withSlot(GainSlot.ONE, new SlotConfig(slot1))
            .withSlot(GainSlot.TWO, new SlotConfig(slot2))
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
            Gains.builder().withKP(1).withKG(2).build(), GravityType.CONSTANT, GainSlot.TWO);

    assertEquals(2, slot.SlotNumber);
    assertEquals(1.0, slot.kP);
    assertEquals(2.0, slot.kG);
    assertEquals(GravityTypeValue.Elevator_Static, slot.GravityType);
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

  /**
   * The requests MotorIOTalonFX doesn't support until differential control is configured and
   * verified.
   */
  private static final Set<Class<?>> DIFFERENTIAL_REQUESTS =
      Set.of(
          Differential.class,
          FollowerRequest.DifferentialFollow.class,
          FollowerRequest.DifferentialStrictFollow.class);

  @Test
  public void shouldRejectDifferentialRequests() throws ReflectiveOperationException {
    for (Class<?> type : DIFFERENTIAL_REQUESTS) {
      MotorRequest request = (MotorRequest) IOTestSupport.sampleRecord(type);
      UnsupportedOperationException error =
          assertThrows(
              UnsupportedOperationException.class,
              () -> TalonFXConversions.toControlRequest(request),
              type.getSimpleName());
      assertTrue(error.getMessage().startsWith(type.getSimpleName()), error.getMessage());
    }
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
