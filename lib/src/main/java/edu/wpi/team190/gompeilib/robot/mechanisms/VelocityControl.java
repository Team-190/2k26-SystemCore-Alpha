package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLib;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.motor.outputs.VelocityRequest;
import java.util.Objects;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

/**
 * Closed-loop velocity control, in the mechanism's velocity unit. The loop can drive a voltage
 * output, torque-current output, or duty-cycle output, with optional feedforward and profiling.
 *
 * @param <V> the mechanism's velocity unit
 */
public final class VelocityControl<V extends Unit> implements MechanismCore.Control {
  private final MechanismCore core;
  private final MechanismConversion<?, V, ?> conversion;
  private final Supplier<? extends Measure<V>> tolerance;
  private final boolean foc;
  private final GainSlot voltageSlot;
  private final GainSlot currentSlot;

  @Getter private Setpoint<V> goal;
  @Getter private Current feedforward = Amps.of(0.0);
  @Getter private Voltage voltageFeedforward = Volts.of(0.0);
  @Getter private double dutyCycleFeedforward = 0.0;
  @Getter @Setter private OutputType outputType;
  @Getter @Setter private boolean profiled = false;

  /**
   * Creates a velocity control with separate voltage and current gain slots, defaulting to voltage.
   *
   * @param core the mechanism's core
   * @param conversion the mechanism's conversion
   * @param goal the initial goal
   * @param tolerance how close the velocity must be to count as at the goal, read each check
   * @param foc whether voltage requests use field-oriented control
   * @param voltageSlot the gain slot for the voltage loop
   * @param currentSlot the gain slot for the torque-current loop
   */
  public VelocityControl(
      MechanismCore core,
      MechanismConversion<?, V, ?> conversion,
      Setpoint<V> goal,
      Supplier<? extends Measure<V>> tolerance,
      boolean foc,
      GainSlot voltageSlot,
      GainSlot currentSlot) {
    this(core, conversion, goal, tolerance, foc, OutputType.VOLTAGE, voltageSlot, currentSlot);
  }

  /**
   * Creates a velocity control with an explicit output type and separate voltage and current gain
   * slots.
   *
   * @param core the mechanism's core
   * @param conversion the mechanism's conversion
   * @param goal the initial goal
   * @param tolerance how close the velocity must be to count as at the goal, read each check
   * @param foc whether voltage requests use field-oriented control
   * @param outputType the initial output type
   * @param voltageSlot the gain slot for the voltage loop
   * @param currentSlot the gain slot for the torque-current loop
   */
  public VelocityControl(
      MechanismCore core,
      MechanismConversion<?, V, ?> conversion,
      Setpoint<V> goal,
      Supplier<? extends Measure<V>> tolerance,
      boolean foc,
      OutputType outputType,
      GainSlot voltageSlot,
      GainSlot currentSlot) {
    this.core = core;
    this.conversion = Objects.requireNonNull(conversion, "conversion");
    this.goal = Objects.requireNonNull(goal, "goal");
    this.tolerance = Objects.requireNonNull(tolerance, "tolerance");
    this.foc = foc;
    this.outputType = Objects.requireNonNull(outputType, "outputType");
    this.voltageSlot = Objects.requireNonNull(voltageSlot, "voltageSlot");
    this.currentSlot = Objects.requireNonNull(currentSlot, "currentSlot");
    core.add(this);
  }

  /**
   * Creates a velocity control using a single gain slot for all output types.
   *
   * @param core the mechanism's core
   * @param conversion the mechanism's conversion
   * @param goal the initial goal
   * @param tolerance how close the velocity must be to count as at the goal, read each check
   * @param foc whether voltage requests use field-oriented control
   * @param outputType the initial output type
   * @param gainSlot the gain slot to use
   */
  public VelocityControl(
      MechanismCore core,
      MechanismConversion<?, V, ?> conversion,
      Setpoint<V> goal,
      Supplier<? extends Measure<V>> tolerance,
      boolean foc,
      OutputType outputType,
      GainSlot gainSlot) {
    this(core, conversion, goal, tolerance, foc, outputType, gainSlot, gainSlot);
  }

  /**
   * Sets the goal, preserving the active output type, and makes this the active control.
   *
   * @param velocity the goal
   */
  public void setGoal(Measure<V> velocity) {
    goal.setSetpoint(velocity);
    core.activate(this);
  }

  /**
   * Replaces the goal, preserving the active output type, and makes this the active control.
   *
   * @param goal the goal
   */
  public void setGoal(Setpoint<V> goal) {
    this.goal = Objects.requireNonNull(goal, "goal");
    core.activate(this);
  }

  /**
   * Sets the goal, runs the voltage loop with a feedforward, and makes this the active control.
   *
   * @param velocity the goal
   * @param feedforward the voltage feedforward
   */
  public void setGoal(Measure<V> velocity, Voltage feedforward) {
    goal.setSetpoint(velocity);
    runVoltage(feedforward);
  }

  /**
   * Replaces the goal, runs the voltage loop with a feedforward, and makes this the active control.
   *
   * @param goal the goal
   * @param feedforward the voltage feedforward
   */
  public void setGoal(Setpoint<V> goal, Voltage feedforward) {
    this.goal = Objects.requireNonNull(goal, "goal");
    runVoltage(feedforward);
  }

  /**
   * Sets the goal, runs the torque-current loop with a feedforward, and makes this the active
   * control.
   *
   * @param velocity the goal
   * @param feedforward the torque-current feedforward
   */
  public void setGoal(Measure<V> velocity, Current feedforward) {
    goal.setSetpoint(velocity);
    runCurrent(feedforward);
  }

  /**
   * Replaces the goal, runs the torque-current loop with a feedforward, and makes this the active
   * control.
   *
   * @param goal the goal
   * @param feedforward the torque-current feedforward
   */
  public void setGoal(Setpoint<V> goal, Current feedforward) {
    this.goal = Objects.requireNonNull(goal, "goal");
    runCurrent(feedforward);
  }

  /**
   * Sets the goal, runs the duty-cycle loop with a feedforward, and makes this the active control.
   *
   * @param velocity the goal
   * @param feedforward the duty-cycle feedforward in [-1, 1]
   */
  public void setGoal(Measure<V> velocity, double feedforward) {
    goal.setSetpoint(velocity);
    runDutyCycle(feedforward);
  }

  /**
   * Replaces the goal, runs the duty-cycle loop with a feedforward, and makes this the active
   * control.
   *
   * @param goal the goal
   * @param feedforward the duty-cycle feedforward in [-1, 1]
   */
  public void setGoal(Setpoint<V> goal, double feedforward) {
    this.goal = Objects.requireNonNull(goal, "goal");
    runDutyCycle(feedforward);
  }

  private void runVoltage(Voltage feedforward) {
    this.voltageFeedforward = Objects.requireNonNull(feedforward, "feedforward");
    outputType = OutputType.VOLTAGE;
    core.activate(this);
  }

  private void runCurrent(Current feedforward) {
    this.feedforward = Objects.requireNonNull(feedforward, "feedforward");
    outputType = OutputType.CURRENT;
    core.activate(this);
  }

  private void runDutyCycle(double feedforward) {
    this.dutyCycleFeedforward = feedforward;
    outputType = OutputType.DUTY_CYCLE;
    core.activate(this);
  }

  /**
   * Returns the leader's velocity, in the mechanism's unit.
   *
   * @return the velocity
   */
  public Measure<V> getVelocity() {
    return conversion.fromMotorVelocity(core.getMotors().leader().velocity);
  }

  public boolean atGoal() {
    return atGoal(goal.getNewSetpoint());
  }

  public boolean atGoal(Measure<V> reference) {
    return Math.abs(getVelocity().baseUnitMagnitude() - reference.baseUnitMagnitude())
        <= tolerance.get().baseUnitMagnitude();
  }

  /**
   * Waits one loop, so periodic applies the latest goal, then until the velocity is at the goal.
   *
   * @return the command
   */
  public Command waitUntilAtGoal() {
    return Commands.waitSeconds(GompeiLib.getLoopPeriod())
        .andThen(Commands.waitUntil(this::atGoal));
  }

  @Override
  public String name() {
    return switch (outputType) {
      case CURRENT -> "VelocityCurrent";
      case DUTY_CYCLE -> "VelocityDutyCycle";
      case VOLTAGE -> "VelocityVoltage";
    };
  }

  @Override
  public MotorRequest request() {
    AngularVelocity velocity = conversion.toMotorVelocity(goal.getNewSetpoint());
    AngularAcceleration acceleration = RadiansPerSecondPerSecond.of(0.0);
    return switch (outputType) {
      case CURRENT ->
          profiled
              ? new VelocityRequest.ProfiledVelocityCurrent(
                  velocity, acceleration, feedforward, currentSlot, RequestOptions.DEFAULT)
              : new VelocityRequest.VelocityCurrent(
                  velocity, acceleration, feedforward, currentSlot, RequestOptions.DEFAULT);
      case DUTY_CYCLE ->
          profiled
              ? new VelocityRequest.ProfiledVelocityDutyCycle(
                  velocity,
                  acceleration,
                  dutyCycleFeedforward,
                  foc,
                  voltageSlot,
                  RequestOptions.DEFAULT)
              : new VelocityRequest.VelocityDutyCycle(
                  velocity,
                  acceleration,
                  dutyCycleFeedforward,
                  foc,
                  voltageSlot,
                  RequestOptions.DEFAULT);
      case VOLTAGE ->
          profiled
              ? new VelocityRequest.ProfiledVelocityVoltage(
                  velocity,
                  acceleration,
                  voltageFeedforward,
                  foc,
                  voltageSlot,
                  RequestOptions.DEFAULT)
              : new VelocityRequest.VelocityVoltage(
                  velocity,
                  acceleration,
                  voltageFeedforward,
                  foc,
                  voltageSlot,
                  RequestOptions.DEFAULT);
    };
  }

  @Override
  public void periodic(String topic) {
    Logger.recordOutputMeasure(topic + "/Velocity/Goal", goal.getSetpoint());
    Logger.recordOutputMeasure(topic + "/Velocity/Offset", goal.getOffset());
    Logger.recordOutputMeasure(topic + "/Velocity/Measured", getVelocity());
    if (outputType == OutputType.CURRENT) {
      Logger.recordOutputMeasure(topic + "/Velocity/Feedforward", feedforward);
    } else if (outputType == OutputType.VOLTAGE) {
      Logger.recordOutputMeasure(topic + "/Velocity/Feedforward", voltageFeedforward);
    } else {
      Logger.recordOutput(topic + "/Velocity/Feedforward", dutyCycleFeedforward);
    }
    Logger.recordOutput(topic + "/Velocity/AtGoal", atGoal());
    Logger.recordOutput(topic + "/Velocity/OutputType", outputType.name());
  }
}
