package edu.wpi.team190.gompeilib.robot.mechanisms;

import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.io.motor.MotorControllerGroup;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import java.util.List;
import org.wpilib.units.measure.Current;

/**
 * A mechanism built on a {@link MechanismCore}. Every other capability extends this one; what a
 * mechanism can be told to do is the set of capabilities it implements.
 */
public interface Mechanism {
  /**
   * Returns the mechanism's core.
   *
   * @return the core
   */
  MechanismCore core();

  /**
   * Runs the mechanism for one loop. Implementations call {@code core().periodic()}, and annotate
   * the method with {@code @Trace} so each mechanism type is timed under its own name.
   */
  void periodic();

  /** Sends neutral every loop until a goal is set. */
  default void stop() {
    core().stop();
  }

  /** Stops sending requests; the last request sent stays in effect. */
  default void idle() {
    core().idle();
  }

  /**
   * Returns the active control's name, or idle or stop.
   *
   * @return the mode
   */
  default String getMode() {
    return core().getMode();
  }

  /**
   * Returns the mechanism's motors, for anything the capabilities don't cover.
   *
   * @return the motors
   */
  default MotorControllerGroup motors() {
    return core().getMotors();
  }

  default boolean isConnected() {
    return motors().isConnected();
  }

  default void clearStickyFaults() {
    motors().clearStickyFaults();
  }

  default void updateCurrentLimits(CurrentLimits currentLimits) {
    motors().updateCurrentLimits(currentLimits);
  }

  default void updateGains(Gains gains, GainSlot slot) {
    motors().updateGains(gains, slot);
  }

  /**
   * Returns the leader's torque current.
   *
   * @return the torque current
   */
  default Current getTorqueCurrent() {
    return motors().leader().torqueCurrent;
  }

  /**
   * Returns each motor's torque current, leader first.
   *
   * @return the torque currents
   */
  default List<Current> getTorqueCurrents() {
    return motors().telemetry().stream().map((MotorTelemetry t) -> t.torqueCurrent).toList();
  }
}
