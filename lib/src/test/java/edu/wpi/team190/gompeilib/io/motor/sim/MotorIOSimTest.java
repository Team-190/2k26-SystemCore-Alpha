package edu.wpi.team190.gompeilib.io.motor.sim;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.TestGains;
import edu.wpi.team190.gompeilib.io.IOTestSupport;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.GravityType;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorAlignment;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorDirection;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.io.motor.config.SoftLimits;
import edu.wpi.team190.gompeilib.io.motor.config.TorqueCurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorControlMode;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetryAutoLogged;
import edu.wpi.team190.gompeilib.io.motor.inputs.RotorPolarity;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.AxisRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.Differential;
import edu.wpi.team190.gompeilib.io.motor.outputs.FollowerRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.NeutralRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.PositionRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.motor.outputs.VelocityRequest;
import edu.wpi.team190.gompeilib.io.physics.DCMotorMechanismSim;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.RoboRioSim;

public class MotorIOSimTest extends GompeiLibTestBase {
  private static final double GEAR_RATIO = 5.0;

  private MechanismSim mechanism;
  private MotorTelemetryAutoLogged telemetry;

  @BeforeEach
  public void setUp() {
    setEnabled(true);
    RoboRioSim.setVInVoltage(12.0);
    mechanism =
        new DCMotorMechanismSim(
            DCMotor.getKrakenX60(1), KilogramSquareMeters.of(0.001), GEAR_RATIO);
    telemetry = new MotorTelemetryAutoLogged();
  }

  @AfterEach
  public void tearDown() {
    setEnabled(false);
    RoboRioSim.resetData();
  }

  private static void setEnabled(boolean enabled) {
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
  }

  private MotorConfig.MotorConfigBuilder config() {
    return MotorConfig.builder()
        .withCanId(1)
        .withBus(CANBusId.named("rio"))
        .withGearRatio(GEAR_RATIO)
        .withCurrentLimits(
            CurrentLimits.fromDoubles()
                .withSupplyCurrentLimit(40.0)
                .withStatorCurrentLimit(80.0)
                .build());
  }

  private MotorIOSim motor(MotorConfig config) {
    return new MotorIOSim(config, mechanism);
  }

  private void run(MotorIOSim motor, int loops) {
    for (int i = 0; i < loops; i++) {
      motor.updateInputs(telemetry);
    }
  }

  @Test
  public void testFactory() {
    MotorIOFactory factory = MotorIOSim.factory(mechanism);
    MotorIOFactory other =
        MotorIOSim.factory(
            new DCMotorMechanismSim(DCMotor.getKrakenX60(1), KilogramSquareMeters.of(0.001), 1.0));

    assertTrue(factory.family().startsWith("Sim@"));
    assertNotEquals(factory.family(), other.family());
    assertTrue(factory.supportsHardwareFollowing());
    assertInstanceOf(MotorIOSim.class, factory.create(config().build()));
  }

  @Test
  public void testFirstMotorDrivesTheMechanism() {
    MotorIOSim leader = motor(config().build());
    MotorIOSim follower = motor(config().withCanId(2).build());

    assertTrue(leader.isDriver());
    assertFalse(follower.isDriver());
    assertTrue(leader.isConnected());
  }

  @Test
  public void testVoltageOutputSpinsTheMechanism() {
    MotorIOSim motor = motor(config().build());
    motor.setControl(
        new OpenLoopRequest.VoltageOutput(Volts.of(6.0), false, RequestOptions.DEFAULT));

    run(motor, 25);

    assertTrue(telemetry.connected);
    assertEquals(6.0, telemetry.appliedVoltage.in(Volts), 1e-9);
    assertEquals(0.5, telemetry.dutyCycle, 1e-9);
    assertEquals(12.0, telemetry.supplyVoltage.in(Volts), 1e-9);
    assertEquals(MotorControlMode.VOLTAGE_OUTPUT, telemetry.controlMode);
    assertFalse(telemetry.focActive);
    assertTrue(telemetry.robotEnabled);
    assertTrue(telemetry.deviceEnabled);
    assertTrue(telemetry.velocity.in(RotationsPerSecond) > 0);
    assertEquals(
        telemetry.position.in(Rotations) * GEAR_RATIO, telemetry.rotorPosition.in(Rotations), 1e-9);
    assertEquals(
        telemetry.velocity.in(RotationsPerSecond) * GEAR_RATIO,
        telemetry.rotorVelocity.in(RotationsPerSecond),
        1e-9);
    assertEquals(
        telemetry.acceleration.in(RotationsPerSecondPerSecond) * GEAR_RATIO,
        telemetry.rotorAcceleration.in(RotationsPerSecondPerSecond),
        1e-9);
    assertEquals(mechanism.currentDraw().in(Amps), telemetry.outputCurrent.in(Amps), 1e-9);
    assertEquals(telemetry.outputCurrent, telemetry.torqueCurrent);
    assertEquals(
        Math.abs(telemetry.outputCurrent.in(Amps) * 0.5), telemetry.supplyCurrent.in(Amps), 1e-9);
    assertEquals(RotorPolarity.COUNTER_CLOCKWISE_POSITIVE, telemetry.rotorPolarity);
    DCMotor gearbox = mechanism.gearbox();
    assertEquals(gearbox.Kt, telemetry.motorKT.in(NewtonMeters.per(Amp)), 1e-9);
    assertEquals(gearbox.Kv, telemetry.motorKV.in(RadiansPerSecond.per(Volt)), 1e-9);
    assertEquals(gearbox.stallCurrent, telemetry.motorStallCurrent.in(Amps), 1e-9);
  }

  @Test
  public void testClockwiseDirectionIsReported() {
    MotorIOSim motor = motor(config().withDirection(MotorDirection.CLOCKWISE_POSITIVE).build());

    run(motor, 1);

    assertEquals(RotorPolarity.CLOCKWISE_POSITIVE, telemetry.rotorPolarity);
  }

  @Test
  public void testOutputsNothingWhileDisabled() {
    MotorIOSim motor = motor(config().build());
    motor.setControl(new OpenLoopRequest.VoltageOutput(Volts.of(6.0)));
    setEnabled(false);

    run(motor, 5);

    assertEquals(0.0, telemetry.appliedVoltage.in(Volts));
    assertEquals(MotorControlMode.DISABLED, telemetry.controlMode);
    assertFalse(telemetry.focActive);
    assertFalse(telemetry.robotEnabled);
    assertEquals(0.0, telemetry.velocity.in(RotationsPerSecond), 1e-9);
  }

  @Test
  public void testDutyCycleScalesWithBattery() {
    MotorIOSim motor = motor(config().build());
    motor.setControl(new OpenLoopRequest.DutyCycleOutput(0.5));
    RoboRioSim.setVInVoltage(10.0);

    run(motor, 1);

    assertEquals(5.0, telemetry.appliedVoltage.in(Volts), 1e-9);
    assertTrue(telemetry.focActive);
  }

  @Test
  public void testDeadBatteryReportsZeroDutyCycle() {
    MotorIOSim motor = motor(config().build());
    motor.setControl(new OpenLoopRequest.VoltageOutput(Volts.of(6.0)));
    RoboRioSim.setVInVoltage(0.0);

    run(motor, 1);

    assertEquals(0.0, telemetry.appliedVoltage.in(Volts));
    assertEquals(0.0, telemetry.dutyCycle);
  }

  @Test
  public void testVoltageIsClampedToTheBattery() {
    MotorIOSim motor = motor(config().build());
    motor.setControl(new OpenLoopRequest.VoltageOutput(Volts.of(20.0)));

    run(motor, 1);

    assertEquals(12.0, telemetry.appliedVoltage.in(Volts), 1e-9);
  }

  @Test
  public void testCurrentOutputAccountsForBackEmf() {
    MotorIOSim motor = motor(config().build());
    motor.setControl(new OpenLoopRequest.CurrentOutput(Amps.of(5.0)));

    run(motor, 1);
    double stalled = telemetry.appliedVoltage.in(Volts);
    run(motor, 20);

    DCMotor gearbox = mechanism.gearbox();
    assertEquals(5.0 * gearbox.R, stalled, 1e-6);
    assertTrue(telemetry.appliedVoltage.in(Volts) > stalled);
    assertEquals(MotorControlMode.CURRENT_OUTPUT, telemetry.controlMode);
    assertTrue(telemetry.focActive);
  }

  @Test
  public void testPositionLoopConverges() {
    MotorIOSim motor =
        motor(
            config()
                .withSlot(
                    GainSlot.ZERO, SlotConfig.position(TestGains.position(8.0, 0, 0.5, 0, 0, 0, 0)))
                .build());
    motor.setControl(new PositionRequest.PositionVoltage(Rotations.of(1.0)));

    run(motor, 1);
    assertEquals(8.0, telemetry.closedLoopProportionalOutput, 1e-9);
    assertEquals(1.0, telemetry.closedLoopPositionError.in(Rotations), 1e-9);
    run(motor, 300);

    assertEquals(1.0, telemetry.position.in(Rotations), 0.02);
    assertEquals(1.0, telemetry.closedLoopPositionReference.in(Rotations), 1e-9);
    assertEquals(0.0, telemetry.closedLoopVelocityReference.in(RotationsPerSecond));
    assertEquals(MotorControlMode.POSITION_VOLTAGE, telemetry.controlMode);
    assertFalse(telemetry.profileRunning);
    assertEquals(GainSlot.ZERO, telemetry.gainSlot);
  }

  @Test
  public void shouldReadGainsDeclaredPerRadianPerRotation() {
    // 8 V per rotation, declared per radian
    var perRadian =
        Gains.angularPosition(Volts)
            .withKP(Volts.per(Radians).ofNative(8.0 / (2 * Math.PI)))
            .withKV(Volts.per(RadiansPerSecond).ofNative(2.0 / (2 * Math.PI)))
            .build();
    MotorIOSim motor =
        motor(config().withSlot(GainSlot.ZERO, SlotConfig.position(perRadian)).build());
    motor.setControl(
        new PositionRequest.PositionVoltage(
            Rotations.of(1.0),
            RotationsPerSecond.of(1.5),
            Volts.of(0.0),
            false,
            GainSlot.ZERO,
            RequestOptions.DEFAULT));

    run(motor, 1);

    assertEquals(8.0, telemetry.closedLoopProportionalOutput, 1e-9);
    assertEquals(2.0 * 1.5, telemetry.closedLoopFeedforward, 1e-9);
  }

  @Test
  public void testPositionFeedforwardTerms() {
    var gains = TestGains.position(0, 0, 0, 0.1, 2.0, 0.5, 0.3);
    MotorIOSim motor = motor(config().withSlot(GainSlot.ONE, SlotConfig.position(gains)).build());
    motor.setControl(
        new PositionRequest.PositionVoltage(
            Rotations.of(0.0),
            RotationsPerSecond.of(1.5),
            Volts.of(0.25),
            false,
            GainSlot.ONE,
            RequestOptions.DEFAULT));

    run(motor, 1);

    assertEquals(0.1 + 2.0 * 1.5 + 0.3 + 0.25, telemetry.closedLoopFeedforward, 1e-9);
    assertEquals(1.5, telemetry.closedLoopPositionReferenceSlope.in(RotationsPerSecond), 1e-9);
    assertEquals(GainSlot.ONE, telemetry.gainSlot);
  }

  @Test
  public void testCosineGravity() {
    var gains = TestGains.position(0, 0, 0, 0, 0, 0, 1.0);
    MotorIOSim motor =
        motor(
            config()
                .withSlot(GainSlot.ZERO, SlotConfig.position(gains, GravityType.COSINE))
                .build());
    motor.setPosition(Rotations.of(0.25));
    motor.setControl(new PositionRequest.PositionVoltage(Rotations.of(0.25)));

    run(motor, 1);

    assertEquals(Math.cos(Math.PI / 2), telemetry.closedLoopFeedforward, 1e-3);
  }

  @Test
  public void testTrapezoidProfileMovesGradually() {
    MotorIOSim motor =
        motor(
            config()
                .withSlot(GainSlot.ZERO, SlotConfig.position(TestGains.positionKP(10.0)))
                .withMotionProfile(
                    new MotionProfileConfig(
                        RotationsPerSecond.of(2.0),
                        RotationsPerSecondPerSecond.of(4.0),
                        RotationsPerSecondPerSecond.per(Second).of(0.0),
                        Volts.per(RotationsPerSecond).ofNative(0.0),
                        Volts.per(RotationsPerSecondPerSecond).ofNative(0.0)))
                .build());
    motor.setControl(new PositionRequest.ProfiledPositionVoltage(Rotations.of(1.0)));

    run(motor, 1);
    assertTrue(telemetry.profileRunning);
    assertFalse(telemetry.profileAtTarget);
    assertTrue(telemetry.closedLoopPositionReference.in(Rotations) < 0.01);
    assertTrue(telemetry.closedLoopPositionReferenceSlope.in(RotationsPerSecond) > 0);

    // Re-sending the same request continues the profile instead of restarting it.
    motor.setControl(new PositionRequest.ProfiledPositionVoltage(Rotations.of(1.0)));
    run(motor, 100);

    assertTrue(telemetry.profileAtTarget);
    assertEquals(1.0, telemetry.closedLoopPositionReference.in(Rotations), 1e-9);
    assertEquals(MotorControlMode.PROFILED_POSITION_VOLTAGE, telemetry.controlMode);
  }

  @Test
  public void testProfilesWithoutConstraintsJumpToTheTarget() {
    MotorIOSim motor = motor(config().build());

    motor.setControl(new PositionRequest.ProfiledPositionVoltage(Rotations.of(2.0)));
    run(motor, 1);
    assertEquals(2.0, telemetry.closedLoopPositionReference.in(Rotations), 1e-9);
    assertTrue(telemetry.profileAtTarget);

    motor.setControl(new PositionRequest.ExponentialProfiledPositionVoltage(Rotations.of(3.0)));
    run(motor, 1);
    assertEquals(3.0, telemetry.closedLoopPositionReference.in(Rotations), 1e-9);
    assertTrue(telemetry.profileRunning);
  }

  @Test
  public void testExponentialProfileMovesGradually() {
    MotorIOSim motor =
        motor(
            config()
                .withMotionProfile(
                    new MotionProfileConfig(
                        RotationsPerSecond.of(0.0),
                        RotationsPerSecondPerSecond.of(0.0),
                        RotationsPerSecondPerSecond.per(Second).of(0.0),
                        Volts.per(RotationsPerSecond).ofNative(0.5),
                        Volts.per(RotationsPerSecondPerSecond).ofNative(0.1)))
                .build());
    motor.setControl(new PositionRequest.ExponentialProfiledPositionVoltage(Rotations.of(10.0)));

    run(motor, 1);

    double reference = telemetry.closedLoopPositionReference.in(Rotations);
    assertTrue(reference > 0 && reference < 10.0);
    assertEquals(MotorControlMode.EXPONENTIAL_PROFILED_POSITION_VOLTAGE, telemetry.controlMode);
  }

  @Test
  public void testDynamicProfilesUseTheRequestsConstraints() {
    MotorIOSim motor = motor(config().build());

    motor.setControl(
        new PositionRequest.DynamicProfiledPositionVoltage(
            Rotations.of(1.0),
            RotationsPerSecond.of(1.0),
            RotationsPerSecondPerSecond.of(2.0),
            RotationsPerSecondPerSecond.per(Second).of(0.0)));
    run(motor, 1);
    assertEquals(0.04, telemetry.closedLoopPositionReferenceSlope.in(RotationsPerSecond), 1e-6);
    assertEquals(MotorControlMode.PROFILED_POSITION_VOLTAGE, telemetry.controlMode);

    motor.setControl(
        new PositionRequest.DynamicExponentialProfiledPositionVoltage(
            Rotations.of(10.0),
            RotationsPerSecond.of(0.0),
            Volts.per(RotationsPerSecond).ofNative(0.5),
            Volts.per(RotationsPerSecondPerSecond).ofNative(0.1)));
    run(motor, 1);
    double reference = telemetry.closedLoopPositionReference.in(Rotations);
    assertTrue(reference > 0 && reference < 10.0);
    assertEquals(MotorControlMode.EXPONENTIAL_PROFILED_POSITION_VOLTAGE, telemetry.controlMode);
  }

  @Test
  public void testVelocityLoopConverges() {
    double kV = 12.0 / (DCMotor.getKrakenX60(1).freeSpeed / (2 * Math.PI) / GEAR_RATIO);
    MotorIOSim motor =
        motor(
            config()
                .withSlot(
                    GainSlot.ZERO, SlotConfig.velocity(TestGains.velocity(0.5, 0, 0, 0, kV, 0, 0)))
                .build());
    motor.setControl(new VelocityRequest.VelocityVoltage(RotationsPerSecond.of(5.0)));

    run(motor, 200);

    assertEquals(5.0, telemetry.velocity.in(RotationsPerSecond), 0.1);
    assertEquals(5.0, telemetry.closedLoopVelocityReference.in(RotationsPerSecond), 1e-9);
    assertEquals(0.0, telemetry.closedLoopPositionReference.in(Rotations));
    assertEquals(MotorControlMode.VELOCITY_VOLTAGE, telemetry.controlMode);
  }

  @Test
  public void testProfiledVelocityRamps() {
    MotorIOSim motor = motor(config().build());
    motor.setControl(
        new VelocityRequest.ProfiledVelocityVoltage(
            RotationsPerSecond.of(1.0),
            RotationsPerSecondPerSecond.of(10.0),
            Volts.of(0.0),
            true,
            GainSlot.ZERO,
            RequestOptions.DEFAULT));

    run(motor, 1);
    assertEquals(0.2, telemetry.closedLoopVelocityReference.in(RotationsPerSecond), 1e-9);
    assertEquals(
        10.0, telemetry.closedLoopVelocityReferenceSlope.in(RotationsPerSecondPerSecond), 1e-9);
    assertTrue(telemetry.profileRunning);
    assertFalse(telemetry.profileAtTarget);

    run(motor, 10);
    assertTrue(telemetry.profileAtTarget);
    assertEquals(0.0, telemetry.closedLoopVelocityReferenceSlope.in(RotationsPerSecondPerSecond));
  }

  @Test
  public void testProfiledVelocityFallsBackToConfiguredAcceleration() {
    MotorIOSim motor =
        motor(
            config()
                .withMotionProfile(
                    new MotionProfileConfig(
                        RotationsPerSecond.of(0.0),
                        RotationsPerSecondPerSecond.of(5.0),
                        RotationsPerSecondPerSecond.per(Second).of(0.0),
                        Volts.per(RotationsPerSecond).ofNative(0.0),
                        Volts.per(RotationsPerSecondPerSecond).ofNative(0.0)))
                .build());
    motor.setControl(new VelocityRequest.ProfiledVelocityVoltage(RotationsPerSecond.of(1.0)));

    run(motor, 1);

    assertEquals(0.1, telemetry.closedLoopVelocityReference.in(RotationsPerSecond), 1e-9);
  }

  @Test
  public void testProfiledVelocityWithoutAccelerationJumps() {
    MotorIOSim motor = motor(config().build());
    motor.setControl(new VelocityRequest.ProfiledVelocityVoltage(RotationsPerSecond.of(3.0)));

    run(motor, 1);

    assertEquals(3.0, telemetry.closedLoopVelocityReference.in(RotationsPerSecond), 1e-9);
    assertTrue(telemetry.profileAtTarget);
    assertEquals(0.0, telemetry.closedLoopVelocityReferenceSlope.in(RotationsPerSecondPerSecond));
  }

  @Test
  public void testIntegralCarriesAcrossTheSameLoopOnly() {
    MotorIOSim motor =
        motor(
            config()
                .withSlot(
                    GainSlot.ZERO, SlotConfig.position(TestGains.position(0, 1.0, 0, 0, 0, 0, 0)))
                .withSlot(
                    GainSlot.ONE, SlotConfig.position(TestGains.position(0, 1.0, 0, 0, 0, 0, 0)))
                .build());
    MotorRequest slotZero = new PositionRequest.PositionVoltage(Rotations.of(100.0));
    motor.setControl(slotZero);
    run(motor, 5);
    double accumulated = telemetry.closedLoopIntegratedOutput;

    motor.setControl(slotZero);
    run(motor, 1);
    assertTrue(telemetry.closedLoopIntegratedOutput > accumulated);

    motor.setControl(
        new PositionRequest.PositionVoltage(
            Rotations.of(100.0),
            RotationsPerSecond.of(0.0),
            Volts.of(0.0),
            true,
            GainSlot.ONE,
            RequestOptions.DEFAULT));
    run(motor, 1);
    assertTrue(telemetry.closedLoopIntegratedOutput < accumulated);
  }

  @Test
  public void testVelocityIntegralCarriesAcrossTheSameLoopOnly() {
    MotorIOSim motor =
        motor(
            config()
                .withSlot(
                    GainSlot.ZERO, SlotConfig.velocity(TestGains.velocity(0, 1.0, 0, 0, 0, 0, 0)))
                .build());
    MotorRequest plain = new VelocityRequest.VelocityVoltage(RotationsPerSecond.of(100.0));
    motor.setControl(plain);
    run(motor, 5);
    double accumulated = telemetry.closedLoopIntegratedOutput;

    motor.setControl(plain);
    run(motor, 1);
    assertTrue(telemetry.closedLoopIntegratedOutput > accumulated);

    // A profiled velocity loop is a different loop, so the integral restarts.
    motor.setControl(new VelocityRequest.ProfiledVelocityVoltage(RotationsPerSecond.of(100.0)));
    run(motor, 1);
    assertTrue(telemetry.closedLoopIntegratedOutput < accumulated);

    // Switching from velocity to position is also a different loop.
    motor.setControl(new PositionRequest.PositionVoltage(Rotations.of(100.0)));
    motor.setControl(plain);
    run(motor, 1);
    assertTrue(telemetry.closedLoopIntegratedOutput < accumulated);
  }

  @Test
  public void testProfilesMissingOneConstraintJumpToTheTarget() {
    MotorIOSim motor =
        motor(
            config()
                .withMotionProfile(
                    new MotionProfileConfig(
                        RotationsPerSecond.of(2.0),
                        RotationsPerSecondPerSecond.of(0.0),
                        RotationsPerSecondPerSecond.per(Second).of(0.0),
                        Volts.per(RotationsPerSecond).ofNative(0.5),
                        Volts.per(RotationsPerSecondPerSecond).ofNative(0.0)))
                .build());

    motor.setControl(new PositionRequest.ProfiledPositionVoltage(Rotations.of(2.0)));
    run(motor, 1);
    assertEquals(2.0, telemetry.closedLoopPositionReference.in(Rotations), 1e-9);

    motor.setControl(new PositionRequest.ExponentialProfiledPositionVoltage(Rotations.of(3.0)));
    run(motor, 1);
    assertEquals(3.0, telemetry.closedLoopPositionReference.in(Rotations), 1e-9);
  }

  @Test
  public void testUpdateGainsReplacesASlot() {
    MotorIOSim motor =
        motor(
            config()
                .withSlot(
                    GainSlot.ZERO,
                    SlotConfig.position(
                        TestGains.position(0, 0, 0, 0, 0, 0, 1.0), GravityType.COSINE))
                .build());
    motor.setControl(new PositionRequest.PositionVoltage(Rotations.of(1.0)));

    motor.updateGains(
        SlotConfig.position(TestGains.position(3.0, 0, 0, 0, 0, 0, 1.0), GravityType.COSINE),
        GainSlot.ZERO);
    run(motor, 1);
    assertEquals(3.0, telemetry.closedLoopProportionalOutput, 1e-9);
    // The gravity type travels with the slot's gains.
    assertEquals(1.0, telemetry.closedLoopFeedforward, 1e-3);

    motor.updateGains(SlotConfig.position(TestGains.positionKP(2.0)), GainSlot.TWO);
    motor.setControl(
        new PositionRequest.PositionVoltage(
            Rotations.of(1.0),
            RotationsPerSecond.of(0.0),
            Volts.of(0.0),
            true,
            GainSlot.TWO,
            RequestOptions.DEFAULT));
    run(motor, 1);
    assertTrue(telemetry.closedLoopProportionalOutput > 0);
  }

  @Test
  public void testSetPositionOffsetsTheReading() {
    MotorIOSim motor = motor(config().build());

    motor.setPosition(Rotations.of(10.0));
    run(motor, 1);

    assertEquals(10.0, telemetry.position.in(Rotations), 1e-9);
    assertEquals(50.0, telemetry.rotorPosition.in(Rotations), 1e-9);
  }

  @Test
  public void testFollowersReportTheLeadersVoltage() {
    MotorIOSim leader = motor(config().build());
    MotorIOSim aligned = motor(config().withCanId(2).build());
    MotorIOSim opposed = motor(config().withCanId(3).build());
    MotorIOSim strict = motor(config().withCanId(4).build());
    aligned.setControl(new FollowerRequest.Follow(1, MotorAlignment.ALIGNED));
    opposed.setControl(new FollowerRequest.Follow(1, MotorAlignment.OPPOSED));
    strict.setControl(new FollowerRequest.StrictFollow(1));
    leader.setControl(new OpenLoopRequest.VoltageOutput(Volts.of(6.0)));

    leader.updateInputs(telemetry);
    MotorTelemetryAutoLogged alignedTelemetry = new MotorTelemetryAutoLogged();
    MotorTelemetryAutoLogged opposedTelemetry = new MotorTelemetryAutoLogged();
    MotorTelemetryAutoLogged strictTelemetry = new MotorTelemetryAutoLogged();
    aligned.updateInputs(alignedTelemetry);
    opposed.updateInputs(opposedTelemetry);
    strict.updateInputs(strictTelemetry);

    assertEquals(6.0, alignedTelemetry.appliedVoltage.in(Volts), 1e-9);
    assertEquals(-6.0, opposedTelemetry.appliedVoltage.in(Volts), 1e-9);
    assertEquals(6.0, strictTelemetry.appliedVoltage.in(Volts), 1e-9);
    assertEquals(MotorControlMode.FOLLOWER, alignedTelemetry.controlMode);
    assertEquals(telemetry.position, alignedTelemetry.position);
    // Current is split between the four motors on the mechanism.
    assertEquals(
        mechanism.currentDraw().in(Amps) / 4, alignedTelemetry.outputCurrent.in(Amps), 1e-9);
  }

  @Test
  public void testDriverIgnoresAFollowRequest() {
    MotorIOSim motor = motor(config().build());
    motor.setControl(new FollowerRequest.Follow(9, MotorAlignment.ALIGNED));

    run(motor, 1);

    assertEquals(0.0, telemetry.appliedVoltage.in(Volts));
  }

  @Test
  public void testPassengersDoNotStepThePhysics() {
    motor(config().build());
    MotorIOSim passenger = motor(config().withCanId(2).build());
    passenger.setControl(new OpenLoopRequest.VoltageOutput(Volts.of(12.0)));

    run(passenger, 20);

    assertEquals(12.0, telemetry.appliedVoltage.in(Volts), 1e-9);
    assertEquals(0.0, telemetry.velocity.in(RotationsPerSecond), 1e-9);
  }

  @Test
  public void testEverySupportedRequestReportsItsMode() throws ReflectiveOperationException {
    List<Class<?>> requests =
        new java.util.ArrayList<>(IOTestSupport.leafRecords(AxisRequest.class));
    requests.addAll(IOTestSupport.leafRecords(NeutralRequest.class));
    requests.add(FollowerRequest.Follow.class);
    requests.add(FollowerRequest.StrictFollow.class);

    for (Class<?> type : requests) {
      MotorIOSim motor = motor(config().build());
      motor.setControl((MotorRequest) IOTestSupport.sampleRecord(type));
      motor.updateInputs(telemetry);
      assertEquals(expectedMode(type.getSimpleName()), telemetry.controlMode, type.getSimpleName());
    }
  }

  private static MotorControlMode expectedMode(String request) {
    return switch (request) {
      case "Neutral" -> MotorControlMode.NEUTRAL;
      case "Coast" -> MotorControlMode.COAST;
      case "StaticBrake" -> MotorControlMode.STATIC_BRAKE;
      case "Tone" -> MotorControlMode.TONE;
      case "Follow", "StrictFollow" -> MotorControlMode.FOLLOWER;
      default ->
          MotorControlMode.valueOf(
              request
                  .replaceFirst("^Dynamic", "")
                  .replaceAll("([a-z])([A-Z])", "$1_$2")
                  .toUpperCase(Locale.ROOT));
    };
  }

  @Test
  public void testSoftLimitsBlockOnlyTheBlockedDirection() {
    MotorIOSim motor =
        motor(
            config()
                .withSoftLimits(SoftLimits.between(Rotations.of(-1.0), Rotations.of(1.0)))
                .build());

    motor.setPosition(Rotations.of(2.0));
    motor.setControl(
        new OpenLoopRequest.VoltageOutput(Volts.of(6.0), false, RequestOptions.DEFAULT));
    run(motor, 1);
    assertEquals(0.0, telemetry.appliedVoltage.in(Volts), 1e-9);
    motor.setControl(
        new OpenLoopRequest.VoltageOutput(Volts.of(-6.0), false, RequestOptions.DEFAULT));
    run(motor, 1);
    assertEquals(-6.0, telemetry.appliedVoltage.in(Volts), 1e-9);

    motor.setPosition(Rotations.of(-2.0));
    run(motor, 1);
    assertEquals(0.0, telemetry.appliedVoltage.in(Volts), 1e-9);
    motor.setControl(
        new OpenLoopRequest.VoltageOutput(Volts.of(6.0), false, RequestOptions.DEFAULT));
    run(motor, 1);
    assertEquals(6.0, telemetry.appliedVoltage.in(Volts), 1e-9);
  }

  @Test
  public void testContinuousWrapTakesTheShortWay() {
    MotorIOSim motor =
        motor(
            config()
                .withContinuousWrap(true)
                .withSlot(GainSlot.ZERO, SlotConfig.position(TestGains.positionKP(10.0)))
                .build());

    motor.setControl(new PositionRequest.PositionVoltage(Rotations.of(0.9)));
    run(motor, 1);

    assertEquals(-0.1, telemetry.closedLoopPositionReference.in(Rotations), 1e-6);
    assertTrue(telemetry.appliedVoltage.in(Volts) < 0.0);
  }

  @Test
  public void testProfileConstraintsCanChangeWhileRunning() {
    MotorIOSim motor = motor(config().build());
    motor.updateMotionProfile(
        MotionProfileConfig.trapezoidal(
            RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(1.0)));

    motor.setControl(new PositionRequest.ProfiledPositionVoltage(Rotations.of(10.0)));
    run(motor, 1);

    assertTrue(telemetry.profileRunning);
    assertTrue(telemetry.closedLoopPositionReference.in(Rotations) < 0.1);
  }

  @Test
  public void testCurrentOutputRespectsTheTorqueCurrentPeaks() {
    MotorIOSim limited =
        motor(
            config()
                .withTorqueCurrentLimits(new TorqueCurrentLimits(Amps.of(0.0), Amps.of(0.0)))
                .build());
    limited.setControl(new OpenLoopRequest.CurrentOutput(Amps.of(40.0)));
    run(limited, 1);
    assertEquals(0.0, telemetry.appliedVoltage.in(Volts), 1e-9);
  }

  @Test
  public void testDifferentialRequests() {
    MotorIOSim motor = motor(config().build());

    motor.setControl(
        new Differential(
            new OpenLoopRequest.VoltageOutput(Volts.of(1.0)),
            new OpenLoopRequest.VoltageOutput(Volts.of(0.0))));
    run(motor, 1);
    assertEquals(MotorControlMode.VOLTAGE_OUTPUT, telemetry.controlMode);

    motor.setControl(new FollowerRequest.DifferentialFollow(2, MotorAlignment.ALIGNED));
    run(motor, 1);
    assertEquals(MotorControlMode.FOLLOWER, telemetry.controlMode);

    motor.setControl(new FollowerRequest.DifferentialFollow(2, MotorAlignment.OPPOSED));
    run(motor, 1);
    assertEquals(MotorControlMode.FOLLOWER, telemetry.controlMode);

    motor.setControl(new FollowerRequest.DifferentialStrictFollow(2));
    run(motor, 1);
    assertEquals(MotorControlMode.FOLLOWER, telemetry.controlMode);
  }
}
