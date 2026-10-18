package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import java.util.Objects;
import lombok.Getter;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

/**
 * Current-based homing: drives the mechanism into a hard stop until the leader's torque current
 * reaches a threshold, then takes the stop's position as the mechanism's position.
 *
 * @param <P> the mechanism's position unit
 */
public final class CurrentHoming<P extends Unit> {
  /** Homing drives past the soft limits to reach the hard stop. */
  public static final RequestOptions OPTIONS =
      new RequestOptions(
          false, false, false, false, true, false, RequestOptions.DEFAULT_UPDATE_FREQUENCY);

  /**
   * How a mechanism homes.
   *
   * @param voltage the voltage that drives the mechanism into the hard stop; its sign picks the
   *     direction
   * @param currentThreshold the torque current that means the mechanism has reached the hard stop
   * @param currentEpsilon how close to {@code currentThreshold} counts as reaching it
   * @param position the mechanism's position at the hard stop
   * @param <P> the mechanism's position unit
   */
  public record Config<P extends Unit>(
      Voltage voltage, Current currentThreshold, Current currentEpsilon, Measure<P> position) {
    public Config {
      Objects.requireNonNull(voltage, "voltage");
      Objects.requireNonNull(currentThreshold, "currentThreshold");
      Objects.requireNonNull(currentEpsilon, "currentEpsilon");
      Objects.requireNonNull(position, "position");
    }
  }

  private final MechanismCore core;
  private final PositionControl<P> positionControl;
  private final boolean foc;

  @Getter private final Config<P> config;

  /**
   * Creates homing for a mechanism.
   *
   * @param core the mechanism's core
   * @param positionControl the mechanism's position control, whose position homing resets
   * @param config how the mechanism homes
   * @param foc whether requests use field-oriented control
   */
  public CurrentHoming(
      MechanismCore core, PositionControl<P> positionControl, Config<P> config, boolean foc) {
    this.core = Objects.requireNonNull(core, "core");
    this.positionControl = Objects.requireNonNull(positionControl, "positionControl");
    this.config = Objects.requireNonNull(config, "config");
    this.foc = foc;
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
   * Idles the mechanism, drives it into the hard stop ignoring soft limits until {@link
   * #atHardStop()}, then sends 0 V and resets its position to the stop.
   *
   * @return the homing command
   */
  public Command command() {
    return Commands.sequence(
        Commands.runOnce(core::idle),
        Commands.run(
                () ->
                    core.setControl(
                        new OpenLoopRequest.VoltageOutput(config.voltage(), foc, OPTIONS)))
            .until(this::atHardStop),
        Commands.runOnce(
            () -> {
              core.setControl(
                  new OpenLoopRequest.VoltageOutput(Volts.zero(), foc, RequestOptions.DEFAULT));
              positionControl.setPosition(config.position());
            }));
  }
}
