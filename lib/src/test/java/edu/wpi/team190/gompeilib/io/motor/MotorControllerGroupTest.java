package edu.wpi.team190.gompeilib.io.motor;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.MotorControllerGroup.FollowMode;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.FollowerMotor;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.MotorAlignment;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorDirection;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.FollowerRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.NeutralRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.PositionRequest;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;

public class MotorControllerGroupTest extends GompeiLibTestBase {
  private static final CANBusId BUS = CANBusId.named("rio");
  private static final CANBusId OTHER_BUS = CANBusId.named("canivore");
  private static final MotorRequest VOLTAGE = new OpenLoopRequest.VoltageOutput(Volts.of(6.0));
  private static final MotorRequest POSITION =
      new PositionRequest.PositionVoltage(Rotations.of(1.0));

  /** Builds a mock motor per config and remembers which config built which motor. */
  private static final class RecordingFactory implements MotorIOFactory {
    final Map<Integer, MotorConfig> configs = new LinkedHashMap<>();
    final Map<Integer, MotorIO> motors = new LinkedHashMap<>();
    private final String family;
    private final boolean hardwareFollowing;

    RecordingFactory(String family, boolean hardwareFollowing) {
      this.family = family;
      this.hardwareFollowing = hardwareFollowing;
    }

    @Override
    public MotorIO create(MotorConfig config) {
      MotorIO motor = mock(MotorIO.class);
      configs.put(config.canId, config);
      motors.put(config.canId, motor);
      return motor;
    }

    @Override
    public String family() {
      return family;
    }

    @Override
    public boolean supportsHardwareFollowing() {
      return hardwareFollowing;
    }
  }

  private static MotorConfig.MotorConfigBuilder leader() {
    return MotorConfig.builder()
        .withCanId(1)
        .withBus(BUS)
        .withDirection(MotorDirection.CLOCKWISE_POSITIVE)
        .withGearRatio(3.0)
        .withCurrentLimits(
            CurrentLimits.fromDoubles()
                .withSupplyCurrentLimit(40.0)
                .withStatorCurrentLimit(40.0)
                .build());
  }

  @Test
  public void testSameBusFollowersHardwareFollow() {
    RecordingFactory factory = new RecordingFactory("Vendor", true);
    MotorControllerGroup group =
        new MotorControllerGroup(
            leader()
                .withFollower(new FollowerMotor(2, MotorAlignment.ALIGNED))
                .withFollower(new FollowerMotor(3, MotorAlignment.OPPOSED))
                .build(),
            factory);

    assertEquals(List.of(FollowMode.HARDWARE, FollowMode.HARDWARE), group.followModes());
    verify(factory.motors.get(2)).setControl(new FollowerRequest.Follow(1, MotorAlignment.ALIGNED));
    verify(factory.motors.get(3)).setControl(new FollowerRequest.Follow(1, MotorAlignment.OPPOSED));
    // Hardware followers keep the leader's direction: alignment comes from the follow request.
    assertEquals(MotorDirection.CLOCKWISE_POSITIVE, factory.configs.get(3).direction);

    group.setControl(POSITION);
    verify(factory.motors.get(1)).setControl(POSITION);
    verify(factory.motors.get(2), never()).setControl(POSITION);
  }

  @Test
  public void testFollowerConfigsInheritTheLeaders() {
    RecordingFactory factory = new RecordingFactory("Vendor", true);
    new MotorControllerGroup(
        leader().withFollower(new FollowerMotor(2, MotorAlignment.ALIGNED)).build(), factory);

    MotorConfig follower = factory.configs.get(2);
    assertEquals(2, follower.canId);
    assertEquals(BUS, follower.bus);
    assertEquals(3.0, follower.gearRatio);
    assertEquals(List.of(), follower.followers);
  }

  @Test
  public void testOtherBusFollowerIsMirrored() {
    RecordingFactory factory = new RecordingFactory("Vendor", true);
    MotorControllerGroup group =
        new MotorControllerGroup(
            leader()
                .withFollower(new FollowerMotor(2, MotorAlignment.OPPOSED).onBus(OTHER_BUS))
                .withFollower(new FollowerMotor(3, MotorAlignment.ALIGNED).onBus(OTHER_BUS))
                .build(),
            factory);

    assertEquals(List.of(FollowMode.MIRROR, FollowMode.MIRROR), group.followModes());
    verify(factory.motors.get(2), never()).setControl(any());
    assertEquals(OTHER_BUS, factory.configs.get(2).bus);
    assertEquals(MotorDirection.COUNTER_CLOCKWISE_POSITIVE, factory.configs.get(2).direction);
    assertEquals(MotorDirection.CLOCKWISE_POSITIVE, factory.configs.get(3).direction);

    group.setControl(VOLTAGE);
    verify(factory.motors.get(1)).setControl(VOLTAGE);
    verify(factory.motors.get(2)).setControl(VOLTAGE);
    verify(factory.motors.get(3)).setControl(VOLTAGE);
  }

  @Test
  public void testMixedVendorFollowerIsMirrored() {
    RecordingFactory leaderFactory = new RecordingFactory("VendorA", true);
    RecordingFactory followerFactory = new RecordingFactory("VendorB", true);
    MotorControllerGroup group =
        new MotorControllerGroup(
            leader()
                .withFollower(new FollowerMotor(2, MotorAlignment.ALIGNED).builtBy(followerFactory))
                .build(),
            leaderFactory);

    assertEquals(List.of(FollowMode.MIRROR), group.followModes());
    assertTrue(followerFactory.motors.containsKey(2));
    assertFalse(leaderFactory.motors.containsKey(2));
  }

  @Test
  public void testFamilyWithoutNativeFollowingIsMirrored() {
    RecordingFactory factory = new RecordingFactory("Vendor", false);
    MotorControllerGroup group =
        new MotorControllerGroup(
            leader().withFollower(new FollowerMotor(2, MotorAlignment.ALIGNED)).build(), factory);

    assertEquals(List.of(FollowMode.MIRROR), group.followModes());
  }

  @Test
  public void testFollowersOnBothBuses() {
    RecordingFactory factory = new RecordingFactory("Vendor", true);
    MotorControllerGroup group =
        new MotorControllerGroup(
            leader()
                .withFollower(new FollowerMotor(2, MotorAlignment.ALIGNED))
                .withFollower(new FollowerMotor(3, MotorAlignment.ALIGNED).onBus(OTHER_BUS))
                .build(),
            factory);

    assertEquals(List.of(FollowMode.HARDWARE, FollowMode.MIRROR), group.followModes());
    group.setControl(VOLTAGE);
    verify(factory.motors.get(2), never()).setControl(VOLTAGE);
    verify(factory.motors.get(3)).setControl(VOLTAGE);
  }

  @Test
  public void shouldRejectClosedLoopWithMirroredFollower() {
    RecordingFactory factory = new RecordingFactory("Vendor", true);
    MotorControllerGroup group =
        new MotorControllerGroup(
            leader()
                .withFollower(new FollowerMotor(2, MotorAlignment.ALIGNED).onBus(OTHER_BUS))
                .build(),
            factory);

    assertThrows(IllegalStateException.class, () -> group.setControl(POSITION));
    verify(factory.motors.get(1), never()).setControl(any());
    verify(factory.motors.get(2), never()).setControl(any());
  }

  @Test
  public void shouldRejectUnlistedRequestWithMirroredFollower() {
    RecordingFactory factory = new RecordingFactory("Vendor", true);
    MotorControllerGroup group =
        new MotorControllerGroup(
            leader()
                .withFollower(new FollowerMotor(2, MotorAlignment.ALIGNED).onBus(OTHER_BUS))
                .build(),
            factory);

    assertThrows(
        IllegalStateException.class,
        () -> group.setControl(new NeutralRequest.Tone(Hertz.of(440.0))));
  }

  @Test
  public void testEveryMirrorableRequestReachesMirroredFollowers() {
    RecordingFactory factory = new RecordingFactory("Vendor", true);
    MotorControllerGroup group =
        new MotorControllerGroup(
            leader()
                .withFollower(new FollowerMotor(2, MotorAlignment.ALIGNED).onBus(OTHER_BUS))
                .build(),
            factory);
    List<MotorRequest> requests =
        List.of(
            new OpenLoopRequest.DutyCycleOutput(0.5),
            VOLTAGE,
            new OpenLoopRequest.CurrentOutput(Amps.of(10.0)),
            new NeutralRequest.Neutral(),
            new NeutralRequest.Coast(),
            new NeutralRequest.StaticBrake());

    for (MotorRequest request : requests) {
      group.setControl(request);
      verify(factory.motors.get(2)).setControl(request);
    }
    assertEquals(requests.size(), MotorControllerGroup.MIRRORABLE_REQUESTS.size());
  }

  @Test
  public void testFanOutAndTelemetry() {
    RecordingFactory factory = new RecordingFactory("Vendor", true);
    MotorControllerGroup group =
        new MotorControllerGroup(
            leader().withFollower(new FollowerMotor(2, MotorAlignment.ALIGNED)).build(), factory);
    MotorIO leaderMotor = factory.motors.get(1);
    MotorIO followerMotor = factory.motors.get(2);
    Gains gains = Gains.builder().withKP(1.0).build();

    group.setPosition(Rotations.of(4.0));
    group.updateGains(gains, GainSlot.ONE);
    group.clearStickyFaults();
    for (MotorIO motor : List.of(leaderMotor, followerMotor)) {
      verify(motor).setPosition(Rotations.of(4.0));
      verify(motor).updateGains(gains, GainSlot.ONE);
      verify(motor).clearStickyFaults();
    }

    doAnswer(
            invocation -> {
              MotorTelemetry telemetry = invocation.getArgument(0);
              telemetry.connected = true;
              telemetry.appliedVoltage = Volts.of(5.0);
              return null;
            })
        .when(leaderMotor)
        .updateInputs(any());
    List<String> keys = new ArrayList<>();
    try (MockedStatic<Logger> logger = mockStatic(Logger.class)) {
      logger
          .when(() -> Logger.processInputs(anyString(), any()))
          .thenAnswer(
              invocation -> {
                keys.add(invocation.getArgument(0));
                return null;
              });
      group.periodic("Intake/Roller");
    }

    assertEquals(List.of("Intake/Roller/Leader", "Intake/Roller/Follower2"), keys);
    assertEquals(Volts.of(5.0), group.leader().appliedVoltage);
    assertEquals(2, group.telemetry().size());
    assertSame(group.leader(), group.telemetry().get(0));
    assertFalse(group.isConnected());

    group.telemetry().get(1).connected = true;
    assertTrue(group.isConnected());
  }
}
