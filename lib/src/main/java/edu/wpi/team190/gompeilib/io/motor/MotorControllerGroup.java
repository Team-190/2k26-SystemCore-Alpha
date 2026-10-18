package edu.wpi.team190.gompeilib.io.motor;

import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
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
   */
  public MotorControllerGroup(MotorConfig config, MotorIOFactory factory) {
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
      MotorConfig followerConfig =
          config.toBuilder()
              .withCanId(follower.canId())
              .withBus(bus)
              .withDirection(direction)
              .clearFollowers()
              .build();

      MotorIO io = followerFactory.create(followerConfig);
      if (hardware) {
        io.setControl(new FollowerRequest.Follow(config.canId, follower.alignment()));
      }
      mirrored |= !hardware;
      followers.add(
          new Member(
              "Follower" + follower.canId(),
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
   * Reads every motor and logs each one under {@code prefix}.
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
