package edu.wpi.team190.gompeilib.io.motor.phoenix;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.team190.gompeilib.GompeiLib;
import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.TestGains;
import edu.wpi.team190.gompeilib.io.IOTestSupport;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.GravityType;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorControlMode;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetryAutoLogged;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixOdometryThread;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixUtil;
import java.util.ArrayDeque;
import java.util.List;
import java.util.Queue;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;
import org.wpilib.units.measure.Angle;

public class MotorIOTalonFXTest extends GompeiLibTestBase {

  private static MotorConfig config() {
    return MotorConfig.builder()
        .withCanId(7)
        .withBus(CANBusId.named("rio"))
        .withGearRatio(4.0)
        .withCurrentLimits(
            CurrentLimits.fromDoubles()
                .withSupplyCurrentLimit(40.0)
                .withStatorCurrentLimit(80.0)
                .build())
        .withSlot(
            GainSlot.ONE,
            SlotConfig.position(TestGains.position(0, 0, 0, 0, 0, 0, 1.0), GravityType.COSINE))
        .build();
  }

  /** The mocks a test body can inspect. */
  private record Mocks(
      MockedConstruction<TalonFX> talons,
      MockedStatic<BaseStatusSignal> signals,
      MockedStatic<PhoenixUtil> phoenix) {}

  /** Runs {@code body} with a mocked TalonFX, batched-signal statics, and signal registration. */
  private static void withMockedTalonFX(
      TalonFXConfigurator configurator, ThrowingConsumer<Mocks> body) throws Exception {
    try (MockedConstruction<TalonFX> talons =
            mockConstruction(
                TalonFX.class,
                withSettings().defaultAnswer(IOTestSupport.phoenixSignalAnswer()),
                (mock, context) -> when(mock.getConfigurator()).thenReturn(configurator));
        MockedStatic<BaseStatusSignal> signals = mockStatic(BaseStatusSignal.class);
        MockedStatic<PhoenixUtil> phoenix = mockStatic(PhoenixUtil.class, CALLS_REAL_METHODS)) {
      phoenix
          .when(() -> PhoenixUtil.registerSignals(any(), any(BaseStatusSignal[].class)))
          .thenAnswer(invocation -> null);
      signals
          .when(() -> BaseStatusSignal.isAllGood(any(BaseStatusSignal[].class)))
          .thenReturn(true);
      body.accept(new Mocks(talons, signals, phoenix));
    }
  }

  private interface ThrowingConsumer<T> {
    void accept(T value) throws Exception;
  }

  private static TalonFXConfigurator okConfigurator() {
    TalonFXConfigurator configurator = mock(TalonFXConfigurator.class);
    when(configurator.apply(any(TalonFXConfiguration.class), anyDouble()))
        .thenReturn(StatusCode.OK);
    when(configurator.apply(any(SlotConfigs.class))).thenReturn(StatusCode.OK);
    return configurator;
  }

  @Test
  public void testConfiguresTheDevice() throws Exception {
    TalonFXConfigurator configurator = okConfigurator();
    withMockedTalonFX(
        configurator,
        mocks -> {
          new MotorIOTalonFX(config());
          TalonFX talon = mocks.talons().constructed().get(0);

          ArgumentCaptor<TalonFXConfiguration> applied =
              ArgumentCaptor.forClass(TalonFXConfiguration.class);
          verify(configurator).apply(applied.capture(), eq(0.25));
          assertEquals(4.0, applied.getValue().Feedback.SensorToMechanismRatio);
          verify(talon).optimizeBusUtilization();
        });
  }

  @Test
  public void testRetriesConfigurationUntilOk() throws Exception {
    TalonFXConfigurator configurator = mock(TalonFXConfigurator.class);
    when(configurator.apply(any(TalonFXConfiguration.class), anyDouble()))
        .thenReturn(StatusCode.TxFailed, StatusCode.OK);
    withMockedTalonFX(
        configurator,
        mocks -> {
          new MotorIOTalonFX(config());
          verify(configurator, times(2)).apply(any(TalonFXConfiguration.class), eq(0.25));
        });
  }

  @Test
  public void testSetsSignalRatesAndRegistersPerBus() throws Exception {
    withMockedTalonFX(
        okConfigurator(),
        mocks -> {
          new MotorIOTalonFX(config());
          mocks
              .signals()
              .verify(
                  () ->
                      BaseStatusSignal.setUpdateFrequencyForAll(
                          eq(1.0 / GompeiLib.LOOP_PERIOD_SECS), any(BaseStatusSignal[].class)));
          mocks
              .signals()
              .verify(
                  () ->
                      BaseStatusSignal.setUpdateFrequencyForAll(
                          eq(4.0), any(BaseStatusSignal[].class)));
          mocks
              .phoenix()
              .verify(
                  () ->
                      PhoenixUtil.registerSignals(
                          eq(new CANBus("rio")), any(BaseStatusSignal[].class)));
        });
  }

  @Test
  public void testFillsEveryTelemetryField() throws Exception {
    withMockedTalonFX(
        okConfigurator(),
        mocks -> {
          MotorIOTalonFX io = new MotorIOTalonFX(config());
          MotorTelemetryAutoLogged telemetry = new MotorTelemetryAutoLogged();

          io.updateInputs(telemetry);

          // The mock reports a position loop on the main axis and a velocity loop on the
          // difference axis, so only the other loop's fields stay zero. TalonFX publishes no rotor
          // acceleration.
          assertEquals(
              Set.of(
                  "rotorAcceleration",
                  "closedLoopVelocityReference",
                  "closedLoopVelocityReferenceSlope",
                  "closedLoopVelocityError",
                  "differentialClosedLoopPositionReference",
                  "differentialClosedLoopPositionReferenceSlope",
                  "differentialClosedLoopPositionError",
                  "odometryTimestamps",
                  "odometryPositions"),
              IOTestSupport.fieldsAtDefault(telemetry));
          assertTrue(io.isConnected());
          assertEquals(MotorControlMode.POSITION_VOLTAGE, telemetry.controlMode);
          assertTrue(telemetry.focActive);
          assertEquals(0.25, telemetry.closedLoopPositionReference.in(Rotations), 1e-9);
          assertEquals(MotorControlMode.VELOCITY_CURRENT, telemetry.differentialControlMode);
          assertEquals(
              0.25, telemetry.differentialClosedLoopVelocityReference.in(RotationsPerSecond), 1e-9);
          assertEquals(GainSlot.ONE, telemetry.gainSlot);
          assertEquals((1L << TalonFXFault.values().length) - 1, telemetry.faults);
          assertEquals(telemetry.faults, telemetry.stickyFaults);
          assertTrue(telemetry.forwardLimitSwitchAsserted);
          assertTrue(telemetry.robotEnabled);
        });
  }

  @Test
  public void testCommandsTheDevice() throws Exception {
    TalonFXConfigurator configurator = okConfigurator();
    withMockedTalonFX(
        configurator,
        mocks -> {
          MotorIOTalonFX io = new MotorIOTalonFX(config());
          TalonFX talon = mocks.talons().constructed().get(0);

          io.setControl(new OpenLoopRequest.VoltageOutput(Volts.of(3.0)));
          ArgumentCaptor<ControlRequest> sent = ArgumentCaptor.forClass(ControlRequest.class);
          verify(talon).setControl(sent.capture());
          assertInstanceOf(VoltageOut.class, sent.getValue());

          io.setPosition(Rotations.of(2.0));
          verify(talon).setPosition(Rotations.of(2.0));

          io.clearStickyFaults();
          verify(talon).clearStickyFaults();

          ArgumentCaptor<SlotConfigs> slots = ArgumentCaptor.forClass(SlotConfigs.class);
          io.updateGains(
              SlotConfig.position(TestGains.positionKP(2.0), GravityType.COSINE), GainSlot.ONE);
          io.updateGains(SlotConfig.position(TestGains.positionKP(3.0)), GainSlot.TWO);
          verify(configurator, times(2)).apply(slots.capture());
          assertEquals(GravityTypeValue.Arm_Cosine, slots.getAllValues().get(0).GravityType);
          assertEquals(1, slots.getAllValues().get(0).SlotNumber);
          assertEquals(2.0, slots.getAllValues().get(0).kP, 1e-9);
          assertEquals(GravityTypeValue.Elevator_Static, slots.getAllValues().get(1).GravityType);
          assertEquals(3.0, slots.getAllValues().get(1).kP, 1e-9);

          io.updateMotionProfile(
              MotionProfileConfig.trapezoidal(
                  RotationsPerSecond.of(2.0), RotationsPerSecondPerSecond.of(4.0)));
          ArgumentCaptor<MotionMagicConfigs> profile =
              ArgumentCaptor.forClass(MotionMagicConfigs.class);
          verify(configurator).apply(profile.capture());
          assertEquals(2.0, profile.getValue().MotionMagicCruiseVelocity);

          io.updateCurrentLimits(new CurrentLimits(null, Amps.of(30.0)));
          ArgumentCaptor<CurrentLimitsConfigs> limits =
              ArgumentCaptor.forClass(CurrentLimitsConfigs.class);
          verify(configurator).apply(limits.capture());
          assertFalse(limits.getValue().SupplyCurrentLimitEnable);
          assertEquals(30.0, limits.getValue().StatorCurrentLimit);
        });
  }

  @Test
  @SuppressWarnings("unchecked")
  public void testSamplesPositionOnTheOdometryThread() throws Exception {
    withMockedTalonFX(
        okConfigurator(),
        mocks -> {
          PhoenixOdometryThread thread = mock(PhoenixOdometryThread.class);
          Queue<Double> timestamps = new ArrayDeque<>(List.of(1.0, 2.0, 3.0));
          Queue<Double> positions = new ArrayDeque<>(List.of(0.5, 0.75));
          when(thread.makeTimestampQueue()).thenReturn(timestamps);
          when(thread.registerSignal(any(StatusSignal.class))).thenReturn(positions);

          MotorIO io = MotorIOTalonFX.factory(thread).create(config());
          MotorTelemetryAutoLogged telemetry = new MotorTelemetryAutoLogged();
          io.updateInputs(telemetry);

          assertArrayEquals(new double[] {1.0, 2.0}, telemetry.odometryTimestamps);
          assertArrayEquals(
              new Angle[] {Rotations.of(0.5), Rotations.of(0.75)}, telemetry.odometryPositions);
          assertEquals(List.of(3.0), List.copyOf(timestamps));
          assertEquals("TalonFX", MotorIOTalonFX.factory(thread).family());
          assertTrue(MotorIOTalonFX.factory(thread).supportsHardwareFollowing());
        });
  }

  @Test
  public void testFactory() throws Exception {
    withMockedTalonFX(
        okConfigurator(),
        mocks -> {
          assertEquals("TalonFX", MotorIOTalonFX.FACTORY.family());
          assertTrue(MotorIOTalonFX.FACTORY.supportsHardwareFollowing());
          assertInstanceOf(MotorIOTalonFX.class, MotorIOTalonFX.FACTORY.create(config()));
        });
  }
}
