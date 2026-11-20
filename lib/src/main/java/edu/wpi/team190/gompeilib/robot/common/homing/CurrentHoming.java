package edu.wpi.team190.gompeilib.robot.common.homing;

import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismCore;
import edu.wpi.team190.gompeilib.robot.common.openloop.OpenLoopControl;
import edu.wpi.team190.gompeilib.robot.common.position.PositionControl;
import java.util.Objects;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

/**
 * Current-based homing: drives the mechanism into a hard stop until the leader's torque current
 * reaches a threshold, then takes the stop's position as the mechanism's position.
 *
 * <p>Homing is a control: while it is active the mode logs {@code Homing}, then {@code Homed} once
 * the stop is found, when it sends zero in the same output until another goal is set. Anything that
 * sets a goal while homing takes over, so the owner of the mechanism leaves it alone while homing.
 *
 * @param <P> the mechanism's position unit
 */
public final class CurrentHoming<P extends Unit> implements MechanismCore.Control {
  /** Homing drives past the soft limits to reach the hard stop. */
  public static final RequestOptions OPTIONS =
      new RequestOptions(
          false, false, false, false, true, false, RequestOptions.DEFAULT_UPDATE_FREQUENCY);

  /**
   * What drives a mechanism into its hard stop: a voltage, or a torque current. Its sign picks the
   * direction.
   */
  public sealed interface Output {
    /**
     * Drives the mechanism with a voltage.
     *
     * @param voltage the voltage
     */
    record Volts(Voltage voltage) implements Output {
      public Volts {
        Objects.requireNonNull(voltage, "voltage");
      }
    }

    /**
     * Drives the mechanism with a torque current.
     *
     * @param current the torque current
     */
    record Amps(Current current) implements Output {
      public Amps {
        Objects.requireNonNull(current, "current");
      }
    }

    static Output volts(Voltage voltage) {
      return new Volts(voltage);
    }

    static Output amps(Current current) {
      return new Amps(current);
    }
  }

  /**
   * How a mechanism homes.
   *
   * @param output what drives the mechanism into the hard stop
   * @param currentThreshold the torque current that means the mechanism has reached the hard stop
   * @param currentEpsilon how close to {@code currentThreshold} counts as reaching it
   * @param position the mechanism's position at the hard stop
   * @param <P> the mechanism's position unit
   */
  public record Config<P extends Unit>(
      Output output, Current currentThreshold, Current currentEpsilon, Measure<P> position) {
    public Config {
      Objects.requireNonNull(output, "output");
      Objects.requireNonNull(currentThreshold, "currentThreshold");
      Objects.requireNonNull(currentEpsilon, "currentEpsilon");
      Objects.requireNonNull(position, "position");
    }
  }

  /** Where homing is. */
  public enum Phase {
    /** Driving into the hard stop. */
    DRIVING,
    /** Stopped: sending zero in the homing output. */
    DONE
  }

  private final MechanismCore core;
  private final PositionControl<P> positionControl;
  private final MotorRequest drive;
  private final MotorRequest zero;

  @Getter private final Config<P> config;
  @Getter private Phase phase = Phase.DONE;

  /**
   * Creates homing for a mechanism and registers it with the core.
   *
   * @param core the mechanism's core
   * @param positionControl the mechanism's position control, whose position homing resets
   * @param voltage the mechanism's voltage control, which builds volt requests
   * @param current the mechanism's current control, which builds torque-current requests
   * @param config how the mechanism homes
   */
  public CurrentHoming(
      MechanismCore core,
      PositionControl<P> positionControl,
      OpenLoopControl<VoltageUnit> voltage,
      OpenLoopControl<CurrentUnit> current,
      Config<P> config) {
    this.core = Objects.requireNonNull(core, "core");
    this.positionControl = Objects.requireNonNull(positionControl, "positionControl");
    Objects.requireNonNull(voltage, "voltage");
    Objects.requireNonNull(current, "current");
    this.config = Objects.requireNonNull(config, "config");
    switch (config.output()) {
      case Output.Volts(Voltage output) -> {
        drive = voltage.request(output, OPTIONS);
        zero = voltage.request(output.times(0.0));
      }
      case Output.Amps(Current output) -> {
        drive = current.request(output, OPTIONS);
        zero = current.request(output.times(0.0));
      }
    }
    core.add(this);
  }

  /**
   * Returns whether the leader's torque current has reached the homing threshold.
   *
   * @return true at the hard stop
   */
  public boolean atHardStop() {
    return core.getMotors()
        .leader()
        .torqueCurrent
        .isNear(config.currentThreshold(), config.currentEpsilon());
  }

  /**
   * Returns a command that drives the mechanism into the hard stop ignoring soft limits until
   * {@link #atHardStop()}, then resets its position to the stop and sends zero in the same output.
   * If interrupted, it stops driving and leaves the position alone.
   *
   * @return the homing command
   */
  public Command command() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  phase = Phase.DRIVING;
                  core.activate(this);
                }),
            Commands.waitUntil(this::atHardStop),
            Commands.runOnce(
                () -> {
                  positionControl.setPosition(config.position());
                  phase = Phase.DONE;
                }))
        .finallyDo(() -> phase = Phase.DONE);
  }

  @Override
  public String name() {
    return phase == Phase.DRIVING ? "Homing" : "Homed";
  }

  @Override
  public MotorRequest request() {
    return phase == Phase.DRIVING ? drive : zero;
  }

  @Override
  public void periodic(String topic) {
    Logger.recordOutput(topic + "/Homing/Phase", phase.name());
    Logger.recordOutput(topic + "/Homing/AtHardStop", atHardStop());
  }
}
