package edu.wpi.team190.gompeilib.robot.common.openloop;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismCore;
import java.util.Objects;
import java.util.function.BiFunction;
import java.util.function.Function;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.DimensionlessUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Dimensionless;
import org.wpilib.units.measure.Voltage;

/**
 * Open-loop control: drives the motors at a goal in volts, torque-current amps, or duty cycle.
 * Build one with {@link #voltage}, {@link #current}, or {@link #dutyCycle}.
 *
 * @param <U> the output unit
 */
public final class OpenLoopControl<U extends Unit> implements MechanismCore.Control {
  /** The default voltage tolerance for {@link #atGoal()}. */
  public static final Voltage VOLTAGE_TOLERANCE = Millivolts.of(500);

  /** The default torque-current tolerance for {@link #atGoal()}. */
  public static final Current CURRENT_TOLERANCE = Milliamps.of(500);

  /** The default duty-cycle tolerance for {@link #atGoal()}. */
  public static final Dimensionless DUTY_CYCLE_TOLERANCE = Value.of(0.05);

  private final MechanismCore core;
  private final String name;
  private final Measure<U> tolerance;
  private final Function<MotorTelemetry, Measure<U>> reading;
  private final BiFunction<Measure<U>, RequestOptions, MotorRequest> requests;

  @Getter private Setpoint<U> goal;

  private OpenLoopControl(
      MechanismCore core,
      String name,
      Setpoint<U> goal,
      Measure<U> tolerance,
      Function<MotorTelemetry, Measure<U>> reading,
      BiFunction<Measure<U>, RequestOptions, MotorRequest> requests) {
    this.core = Objects.requireNonNull(core, "core");
    this.name = name;
    this.goal = Objects.requireNonNull(goal, "goal");
    this.tolerance = Objects.requireNonNull(tolerance, "tolerance");
    this.reading = reading;
    this.requests = requests;
    core.add(this);
  }

  /**
   * Creates a voltage control and registers it with the core.
   *
   * @param core the mechanism's core
   * @param goal the initial goal
   * @param tolerance how close the leader's applied voltage must be to count as at the goal
   * @param foc whether requests use field-oriented control
   * @return the control
   */
  public static OpenLoopControl<VoltageUnit> voltage(
      MechanismCore core, Setpoint<VoltageUnit> goal, Voltage tolerance, boolean foc) {
    return new OpenLoopControl<>(
        core,
        "Voltage",
        goal,
        tolerance,
        t -> t.appliedVoltage,
        (voltage, options) ->
            new OpenLoopRequest.VoltageOutput(Volts.of(voltage.in(Volts)), foc, options));
  }

  /**
   * Creates a voltage control with the default tolerance and registers it with the core.
   *
   * @param core the mechanism's core
   * @param goal the initial goal
   * @param foc whether requests use field-oriented control
   * @return the control
   */
  public static OpenLoopControl<VoltageUnit> voltage(
      MechanismCore core, Setpoint<VoltageUnit> goal, boolean foc) {
    return voltage(core, goal, VOLTAGE_TOLERANCE, foc);
  }

  /**
   * Creates a voltage control with a 0 V goal limited to ±12 V, and registers it with the core.
   *
   * @param core the mechanism's core
   * @param offsetStep how far one increment or decrement moves the goal
   * @param foc whether requests use field-oriented control
   * @return the control
   */
  public static OpenLoopControl<VoltageUnit> voltage(
      MechanismCore core, Voltage offsetStep, boolean foc) {
    return voltage(core, new Setpoint<>(Volts.of(0), offsetStep, Volts.of(-12), Volts.of(12)), foc);
  }

  /**
   * Creates a torque-current control and registers it with the core.
   *
   * @param core the mechanism's core
   * @param goal the initial goal
   * @param tolerance how close the leader's torque current must be to count as at the goal
   * @return the control
   */
  public static OpenLoopControl<CurrentUnit> current(
      MechanismCore core, Setpoint<CurrentUnit> goal, Current tolerance) {
    return new OpenLoopControl<>(
        core,
        "Current",
        goal,
        tolerance,
        t -> t.torqueCurrent,
        (current, options) ->
            new OpenLoopRequest.CurrentOutput(
                Amps.of(current.in(Amps)), 1.0, Amps.of(0.0), options));
  }

  /**
   * Creates a torque-current control with the default tolerance and registers it with the core.
   *
   * @param core the mechanism's core
   * @param goal the initial goal
   * @return the control
   */
  public static OpenLoopControl<CurrentUnit> current(
      MechanismCore core, Setpoint<CurrentUnit> goal) {
    return current(core, goal, CURRENT_TOLERANCE);
  }

  /**
   * Creates a duty-cycle control and registers it with the core.
   *
   * @param core the mechanism's core
   * @param goal the initial goal, in [-1, 1]
   * @param tolerance how close the leader's duty cycle must be to count as at the goal
   * @param foc whether requests use field-oriented control
   * @return the control
   */
  public static OpenLoopControl<DimensionlessUnit> dutyCycle(
      MechanismCore core, Setpoint<DimensionlessUnit> goal, Dimensionless tolerance, boolean foc) {
    return new OpenLoopControl<>(
        core,
        "DutyCycle",
        goal,
        tolerance,
        t -> Value.of(t.dutyCycle),
        (dutyCycle, options) ->
            new OpenLoopRequest.DutyCycleOutput(dutyCycle.in(Value), foc, options));
  }

  /**
   * Creates a duty-cycle control with the default tolerance and registers it with the core.
   *
   * @param core the mechanism's core
   * @param goal the initial goal, in [-1, 1]
   * @param foc whether requests use field-oriented control
   * @return the control
   */
  public static OpenLoopControl<DimensionlessUnit> dutyCycle(
      MechanismCore core, Setpoint<DimensionlessUnit> goal, boolean foc) {
    return dutyCycle(core, goal, DUTY_CYCLE_TOLERANCE, foc);
  }

  /**
   * Creates a duty-cycle control with a 0 goal limited to [-1, 1], and registers it with the core.
   *
   * @param core the mechanism's core
   * @param offsetStep how far one increment or decrement moves the goal
   * @param foc whether requests use field-oriented control
   * @return the control
   */
  public static OpenLoopControl<DimensionlessUnit> dutyCycle(
      MechanismCore core, double offsetStep, boolean foc) {
    return dutyCycle(
        core, new Setpoint<>(Value.of(0), Value.of(offsetStep), Value.of(-1), Value.of(1)), foc);
  }

  /**
   * Returns how an output in a unit becomes an open-loop request: voltage through {@code voltage},
   * torque current through {@code current}.
   *
   * @param unit the output unit
   * @param voltage the voltage control
   * @param current the current control
   * @param options the options each request carries
   * @param <O> the output unit
   * @return the request function
   * @throws IllegalArgumentException if the unit isn't volts or amps
   */
  public static <O extends Unit> Function<Measure<O>, MotorRequest> select(
      O unit,
      OpenLoopControl<VoltageUnit> voltage,
      OpenLoopControl<CurrentUnit> current,
      RequestOptions options) {
    OutputType type =
        OutputType.of(unit)
            .filter(t -> t != OutputType.DUTY_CYCLE)
            .orElseThrow(() -> new IllegalArgumentException("Use volts or amps, not " + unit));
    Objects.requireNonNull(options, "options");
    return type == OutputType.VOLTAGE
        ? output -> voltage.request(Volts.of(output.baseUnitMagnitude()), options)
        : output -> current.request(Amps.of(output.baseUnitMagnitude()), options);
  }

  /**
   * Returns how an output in a unit becomes an open-loop request with the default options.
   *
   * @param unit the output unit
   * @param voltage the voltage control
   * @param current the current control
   * @param <O> the output unit
   * @return the request function
   * @throws IllegalArgumentException if the unit isn't volts or amps
   */
  public static <O extends Unit> Function<Measure<O>, MotorRequest> select(
      O unit, OpenLoopControl<VoltageUnit> voltage, OpenLoopControl<CurrentUnit> current) {
    return select(unit, voltage, current, RequestOptions.DEFAULT);
  }

  /**
   * Sets the goal and makes this the active control.
   *
   * @param output the goal
   */
  public void setGoal(Measure<U> output) {
    goal.setSetpoint(Objects.requireNonNull(output, "output"));
    core.activate(this);
  }

  /**
   * Replaces the goal and makes this the active control.
   *
   * @param goal the goal
   */
  public void setGoal(Setpoint<U> goal) {
    this.goal = Objects.requireNonNull(goal, "goal");
    core.activate(this);
  }

  /**
   * Returns the leader's output, in this control's unit.
   *
   * @return the output
   */
  public Measure<U> getOutput() {
    return reading.apply(core.getMotors().leader());
  }

  public boolean atGoal() {
    return atGoal(goal.getNewSetpoint());
  }

  public boolean atGoal(Measure<U> reference) {
    return Math.abs(getOutput().baseUnitMagnitude() - reference.baseUnitMagnitude())
        <= tolerance.baseUnitMagnitude();
  }

  @Override
  public String name() {
    return name;
  }

  @Override
  public MotorRequest request() {
    return request(goal.getNewSetpoint());
  }

  /**
   * Returns the request that drives the motors at an output, the way this control does.
   *
   * @param output the output
   * @return the request
   */
  public MotorRequest request(Measure<U> output) {
    return request(output, RequestOptions.DEFAULT);
  }

  /**
   * Returns the request that drives the motors at an output with options, the way this control
   * does.
   *
   * @param output the output
   * @param options the request options
   * @return the request
   */
  public MotorRequest request(Measure<U> output, RequestOptions options) {
    return requests.apply(
        Objects.requireNonNull(output, "output"), Objects.requireNonNull(options, "options"));
  }

  @Override
  public void periodic(String topic) {
    Logger.recordOutputMeasure(topic + "/" + name + "/Goal", goal.getSetpoint());
    Logger.recordOutputMeasure(topic + "/" + name + "/Offset", goal.getOffset());
    Logger.recordOutput(topic + "/" + name + "/AtGoal", atGoal());
  }
}
