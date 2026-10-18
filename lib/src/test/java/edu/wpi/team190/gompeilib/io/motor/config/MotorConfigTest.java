package edu.wpi.team190.gompeilib.io.motor.config;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.io.IOTestSupport;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.bus.CANPort;

public class MotorConfigTest extends GompeiLibTestBase {
  private static final CurrentLimits LIMITS =
      CurrentLimits.fromDoubles().withSupplyCurrentLimit(30.0).withStatorCurrentLimit(60.0).build();

  @Test
  public void testDefaults() {
    MotorConfig config =
        MotorConfig.builder()
            .withCanId(3)
            .withBus(CANBusId.named("rio"))
            .withCurrentLimits(LIMITS)
            .build();

    assertEquals(3, config.canId);
    assertEquals(CANBusId.named("rio"), config.bus);
    assertSame(LIMITS, config.currentLimits);
    assertEquals(MotorDirection.COUNTER_CLOCKWISE_POSITIVE, config.direction);
    assertEquals(NeutralMode.BRAKE, config.neutralMode);
    assertEquals(1.0, config.gearRatio);
    assertEquals(Map.of(), config.slots);
    assertSame(MotionProfileConfig.NONE, config.motionProfile);
    assertSame(OpenLoopRamps.NONE, config.openLoopRamps);
    assertTrue(config.enableFOC);
    assertSame(SignalRates.DEFAULT, config.signalRates);
    assertEquals(List.of(), config.followers);
  }

  @Test
  public void testEveryField() {
    SlotConfig slot = new SlotConfig(Gains.builder().withKP(1.0).build(), GravityType.COSINE);
    MotionProfileConfig profile =
        new MotionProfileConfig(
            RotationsPerSecond.of(1.0),
            RotationsPerSecondPerSecond.of(2.0),
            RotationsPerSecondPerSecond.per(Second).of(3.0),
            Volts.per(RotationsPerSecond).ofNative(0.1),
            Volts.per(RotationsPerSecondPerSecond).ofNative(0.2));
    OpenLoopRamps ramps = new OpenLoopRamps(Seconds.of(0.1), Seconds.of(0.2), Seconds.of(0.3));
    SignalRates rates = new SignalRates(Hertz.of(100.0), Hertz.of(10.0), Map.of());
    FollowerMotor follower = new FollowerMotor(4, MotorAlignment.OPPOSED);

    MotorConfig config =
        MotorConfig.builder()
            .withCanId(3)
            .withBus(CANBusId.port(CANPort.CAN_S1))
            .withDirection(MotorDirection.CLOCKWISE_POSITIVE)
            .withNeutralMode(NeutralMode.COAST)
            .withCurrentLimits(LIMITS)
            .withGearRatio(12.5)
            .withSlot(GainSlot.TWO, slot)
            .withMotionProfile(profile)
            .withOpenLoopRamps(ramps)
            .withEnableFOC(false)
            .withSignalRates(rates)
            .withFollower(follower)
            .build();

    assertEquals(CANBusId.port(CANPort.CAN_S1), config.bus);
    assertEquals(MotorDirection.CLOCKWISE_POSITIVE, config.direction);
    assertEquals(NeutralMode.COAST, config.neutralMode);
    assertEquals(12.5, config.gearRatio);
    assertEquals(Map.of(GainSlot.TWO, slot), config.slots);
    assertSame(profile, config.motionProfile);
    assertSame(ramps, config.openLoopRamps);
    assertFalse(config.enableFOC);
    assertSame(rates, config.signalRates);
    assertEquals(List.of(follower), config.followers);
  }

  @Test
  public void testToBuilderCopiesAndOverrides() {
    MotorConfig leader =
        MotorConfig.builder()
            .withCanId(3)
            .withBus(CANBusId.named("rio"))
            .withCurrentLimits(LIMITS)
            .withGearRatio(4.0)
            .withFollower(new FollowerMotor(4, MotorAlignment.ALIGNED))
            .build();

    MotorConfig follower = leader.toBuilder().withCanId(4).clearFollowers().build();

    assertEquals(4, follower.canId);
    assertEquals(4.0, follower.gearRatio);
    assertEquals(List.of(), follower.followers);
  }

  @Test
  public void shouldRequireIdBusAndLimits() {
    assertThrows(
        NullPointerException.class,
        () ->
            MotorConfig.builder().withBus(CANBusId.named("rio")).withCurrentLimits(LIMITS).build());
    assertThrows(
        NullPointerException.class,
        () -> MotorConfig.builder().withCanId(1).withCurrentLimits(LIMITS).build());
    assertThrows(
        NullPointerException.class,
        () -> MotorConfig.builder().withCanId(1).withBus(CANBusId.named("rio")).build());
  }

  @Test
  public void testControlFieldsAreRealTelemetryFields() {
    assertTrue(
        IOTestSupport.publicFieldNames(MotorTelemetry.class)
            .containsAll(MotorConfig.CONTROL_FIELDS));
  }

  @Test
  public void testFollowerOutputSignalsAreControlRate() {
    // A hardware follower copies these from its leader, so they must run at the control rate.
    assertTrue(MotorConfig.CONTROL_FIELDS.contains("dutyCycle"));
    assertTrue(MotorConfig.CONTROL_FIELDS.contains("appliedVoltage"));
    assertTrue(MotorConfig.CONTROL_FIELDS.contains("torqueCurrent"));
  }
}
