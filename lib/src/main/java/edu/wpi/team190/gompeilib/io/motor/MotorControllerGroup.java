package edu.wpi.team190.gompeilib.io.motor;

import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.FeedbackSensor;
import edu.wpi.team190.gompeilib.io.motor.config.FollowerMotor;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorAlignment;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorDirection;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetryAutoLogged;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.FollowerRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.NeutralRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.wpilib.units.measure.Angle;

/**
 * A leader motor and its followers, built from one {@link MotorConfig}. The group is the single
 * entry point a subsystem sends requests through.
 *
 * <p>A follower hardware-follows the leader when it is built by the same controller family, on the
 * same bus, and that family supports native following. Every other follower is mirrored: it is sent
 * the same request as the leader, with its direction flipped in config if it is opposed. Only the
 * open-loop requests in {@link #MIRRORABLE_REQUESTS} can be mirrored; any other request sent to a
 * group with a mirrored follower throws instead of being sent, so two independent closed loops
 * never fight each other.
 */
public class MotorControllerGroup {
  /** The requests that give the same result whether followed or mirrored. */
  public static final Set<Class<? extends MotorRequest>> MIRRORABLE_REQUESTS =
      Set.of(
          OpenLoopRequest.DutyCycleOutput.class,
          OpenLoopRequest.VoltageOutput.class,
          OpenLoopRequest.CurrentOutput.class,
          NeutralRequest.Neutral.class,
          NeutralRequest.Coast.class,
          NeutralRequest.StaticBrake.class);

  /** How a follower tracks the leader. */
  public enum FollowMode {
    /** The follower natively follows the leader. */
    HARDWARE,
    /** The follower is sent the same open-loop requests as the leader. */
    MIRROR;
  }

  private record DeviceAddress(CANBusId bus, int canId) {}

  private record Member(
      String key, int canId, MotorIO io, MotorTelemetryAutoLogged telemetry, FollowMode mode) {}

  private final Member leader;
  private final List<Member> followers = new ArrayList<>();
  private final boolean hasMirroredFollower;

  /**
   * Builds the leader and every follower in {@code config}.
   *
   * @param config the leader's config, including its followers
   * @param factory how to build the leader, and any follower without its own factory
   * @throws IllegalArgumentException if two motors share a CAN ID on the same bus
   */
  public MotorControllerGroup(MotorConfig config, MotorIOFactory factory) {
    Set<DeviceAddress> addresses = new HashSet<>();
    addresses.add(new DeviceAddress(config.bus, config.canId));
    for (FollowerMotor follower : config.followers) {
      DeviceAddress address =
          new DeviceAddress(follower.bus().orElse(config.bus), follower.canId());
      if (!addresses.add(address)) {
        throw new IllegalArgumentException(
            "Follower " + follower.canId() + " shares its CAN ID and bus with another motor");
      }
    }

    leader = new Member("Leader", config.canId, factory.create(config), newTelemetry(), null);

    boolean mirrored = false;
    for (FollowerMotor follower : config.followers) {
      CANBusId bus = follower.bus().orElse(config.bus);
      MotorIOFactory followerFactory = follower.factory().orElse(factory);
      boolean hardware =
          followerFactory.supportsHardwareFollowing()
              && followerFactory.family().equals(factory.family())
              && bus.equals(config.bus);
      MotorDirection direction =
          hardware || follower.alignment() == MotorAlignment.ALIGNED
              ? config.direction
              : config.direction.opposite();
      // A mirrored follower never closes a loop, and the leader's remote encoder may be on another
      // bus or unreadable by another vendor, so it reads its own rotor. The encoder is on the
      // gearbox output, so the gear ratio still maps the rotor to the output.
      FeedbackSensor feedbackSensor = hardware ? config.feedbackSensor : FeedbackSensor.INTERNAL;
      MotorConfig followerConfig =
          config.toBuilder()
              .withCanId(follower.canId())
              .withBus(bus)
              .withDirection(direction)
              .withFeedbackSensor(feedbackSensor)
              .clearFollowers()
              .build();

      MotorIO io = followerFactory.create(followerConfig);
      if (hardware) {
        io.setControl(new FollowerRequest.Follow(config.canId, follower.alignment()));
      }
      mirrored |= !hardware;
      // CAN IDs are only unique per bus, so a follower off the leader's bus is keyed by its bus
      // too.
      String key =
          bus.equals(config.bus)
              ? "Follower" + follower.canId()
              : "Follower" + follower.canId() + "@" + bus.label();
      followers.add(
          new Member(
              key,
              follower.canId(),
              io,
              newTelemetry(),
              hardware ? FollowMode.HARDWARE : FollowMode.MIRROR));
    }
    hasMirroredFollower = mirrored;
  }

  private static MotorTelemetryAutoLogged newTelemetry() {
    return new MotorTelemetryAutoLogged();
  }

  /**
   * Sends a request to the leader, and to every mirrored follower if the request can be mirrored.
   *
   * @param request the request
   * @throws IllegalStateException if the request can't be mirrored and a follower is mirrored;
   *     nothing is sent
   */
  public void setControl(MotorRequest request) {
    boolean mirrorable = MIRRORABLE_REQUESTS.contains(request.getClass());
    if (hasMirroredFollower && !mirrorable) {
      throw new IllegalStateException(
          request.getClass().getSimpleName()
              + " can't be sent to a group with a mirrored follower: only open-loop requests are"
              + " mirrored, so a closed loop would run independently on each motor");
    }
    leader.io().setControl(request);
    if (mirrorable) {
      for (Member follower : followers) {
        if (follower.mode() == FollowMode.MIRROR) {
          follower.io().setControl(request);
        }
      }
    }
  }

  /**
   * Reads every motor and logs each one under {@code prefix}: the leader as {@code Leader}, each
   * follower on the leader's bus as {@code Follower<id>}, and each follower on another bus as
   * {@code Follower<id>@<bus>}.
   *
   * @param prefix the log prefix, e.g. {@code "Intake/Roller"}
   */
  public void periodic(String prefix) {
    for (Member member : members()) {
      member.io().updateInputs(member.telemetry());
      Logger.processInputs(prefix + "/" + member.key(), member.telemetry());
    }
  }

  /**
   * Resets every motor's reported position.
   *
   * @param position the new position
   */
  public void setPosition(Angle position) {
    for (Member member : members()) {
      member.io().setPosition(position);
    }
  }

  /**
   * Updates a gain slot on every motor.
   *
   * @param gains the gains
   * @param slot the slot
   */
  public void updateGains(Gains gains, GainSlot slot) {
    for (Member member : members()) {
      member.io().updateGains(gains, slot);
    }
  }

  /**
   * Updates the stored motion-profile constraints on every motor.
   *
   * @param motionProfile the constraints
   */
  public void updateMotionProfile(MotionProfileConfig motionProfile) {
    for (Member member : members()) {
      member.io().updateMotionProfile(motionProfile);
    }
  }

  /**
   * Updates the current limits on every motor.
   *
   * @param currentLimits the limits
   */
  public void updateCurrentLimits(CurrentLimits currentLimits) {
    for (Member member : members()) {
      member.io().updateCurrentLimits(currentLimits);
    }
  }

  /** Clears sticky faults on every motor. */
  public void clearStickyFaults() {
    for (Member member : members()) {
      member.io().clearStickyFaults();
    }
  }

  /**
   * Returns whether every motor is connected.
   *
   * @return true if all are connected
   */
  public boolean isConnected() {
    for (Member member : members()) {
      if (!member.telemetry().connected) {
        return false;
      }
    }
    return true;
  }

  /**
   * Returns the leader's latest telemetry.
   *
   * @return the leader's telemetry
   */
  public MotorTelemetry leader() {
    return leader.telemetry();
  }

  /**
   * Returns every motor's latest telemetry, leader first, then followers in config order.
   *
   * @return the telemetry
   */
  public List<MotorTelemetry> telemetry() {
    List<MotorTelemetry> all = new ArrayList<>();
    for (Member member : members()) {
      all.add(member.telemetry());
    }
    return Collections.unmodifiableList(all);
  }

  /**
   * Returns how each follower tracks the leader, in config order.
   *
   * @return the follow modes
   */
  public List<FollowMode> followModes() {
    return followers.stream().map(Member::mode).toList();
  }

  private List<Member> members() {
    List<Member> all = new ArrayList<>(followers.size() + 1);
    all.add(leader);
    all.addAll(followers);
    return all;
  }
}
