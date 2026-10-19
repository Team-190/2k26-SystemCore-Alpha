package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.outputs.AxisRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.Differential;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.motor.outputs.PositionRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import java.util.Objects;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

/**
 * Closed-loop differential control for mechanisms driven by two coupled motors on average and
 * difference axes.
 *
 * @param <AvgUnit> the average axis unit
 * @param <DiffUnit> the difference axis unit
 */
public final class DifferentialControl<AvgUnit extends Unit, DiffUnit extends Unit>
    implements MechanismCore.Control {
  /** The motion profile mode for the average axis of differential control. */
  public enum ProfileMode {
    /** Trapezoidal motion profiling (Motion Magic). */
    PROFILED,
    /** Unprofiled closed-loop control. */
    UNPROFILED,
    /** Exponential motion profiling (Motion Magic Expo). */
    EXPONENTIAL
  }

  private final MechanismCore core;
  private final MechanismConversion<AvgUnit, ?, ?> averageConversion;
  private final MechanismConversion<DiffUnit, ?, ?> differenceConversion;
  private final Supplier<? extends Measure<AvgUnit>> averageTolerance;
  private final Supplier<? extends Measure<DiffUnit>> differenceTolerance;
  private final Supplier<? extends Measure<AvgUnit>> averageMeasurement;
  private final Supplier<? extends Measure<DiffUnit>> differenceMeasurement;
  private final boolean foc;

  @Getter private Setpoint<AvgUnit> averageGoal;
  @Getter private Setpoint<DiffUnit> differenceGoal;
  @Getter @Setter private GainSlot averageSlot = GainSlot.ZERO;
  @Getter @Setter private GainSlot differenceSlot = GainSlot.ONE;
  @Getter @Setter private OutputType outputType;
  @Getter @Setter private ProfileMode profileMode = ProfileMode.PROFILED;
  @Getter @Setter private RequestOptions options = RequestOptions.DEFAULT;

  @Getter private Voltage averageVoltageFeedforward = Volts.of(0.0);
  @Getter private Voltage differenceVoltageFeedforward = Volts.of(0.0);
  @Getter private Current averageCurrentFeedforward = Amps.of(0.0);
  @Getter private Current differenceCurrentFeedforward = Amps.of(0.0);
  @Getter private double averageDutyCycleFeedforward = 0.0;
  @Getter private double differenceDutyCycleFeedforward = 0.0;

  /**
   * Creates a differential control and registers it with the core.
   *
   * @param core the mechanism core
   * @param averageConversion conversion for the average axis
   * @param differenceConversion conversion for the difference axis
   * @param averageGoal the initial average goal
   * @param differenceGoal the initial difference goal
   * @param averageTolerance tolerance for average goal checks
   * @param differenceTolerance tolerance for difference goal checks
   * @param averageMeasurement supplier for current average measurement
   * @param differenceMeasurement supplier for current difference measurement
   * @param foc whether requests use field-oriented control
   * @param outputType the closed-loop output type
   */
  public DifferentialControl(
      MechanismCore core,
      MechanismConversion<AvgUnit, ?, ?> averageConversion,
      MechanismConversion<DiffUnit, ?, ?> differenceConversion,
      Setpoint<AvgUnit> averageGoal,
      Setpoint<DiffUnit> differenceGoal,
      Supplier<? extends Measure<AvgUnit>> averageTolerance,
      Supplier<? extends Measure<DiffUnit>> differenceTolerance,
      Supplier<? extends Measure<AvgUnit>> averageMeasurement,
      Supplier<? extends Measure<DiffUnit>> differenceMeasurement,
      boolean foc,
      OutputType outputType) {
    this.core = core;
    this.averageConversion = Objects.requireNonNull(averageConversion, "averageConversion");
    this.differenceConversion =
        Objects.requireNonNull(differenceConversion, "differenceConversion");
    this.averageGoal = Objects.requireNonNull(averageGoal, "averageGoal");
    this.differenceGoal = Objects.requireNonNull(differenceGoal, "differenceGoal");
    this.averageTolerance = Objects.requireNonNull(averageTolerance, "averageTolerance");
    this.differenceTolerance = Objects.requireNonNull(differenceTolerance, "differenceTolerance");
    this.averageMeasurement = Objects.requireNonNull(averageMeasurement, "averageMeasurement");
    this.differenceMeasurement =
        Objects.requireNonNull(differenceMeasurement, "differenceMeasurement");
    this.foc = foc;
    this.outputType = Objects.requireNonNull(outputType, "outputType");
    core.add(this);
  }

  /**
   * Creates a differential control defaulting to voltage output.
   *
   * @param core the mechanism core
   * @param averageConversion conversion for the average axis
   * @param differenceConversion conversion for the difference axis
   * @param averageGoal the initial average goal
   * @param differenceGoal the initial difference goal
   * @param averageTolerance tolerance for average goal checks
   * @param differenceTolerance tolerance for difference goal checks
   * @param averageMeasurement supplier for current average measurement
   * @param differenceMeasurement supplier for current difference measurement
   * @param foc whether requests use field-oriented control
   */
  public DifferentialControl(
      MechanismCore core,
      MechanismConversion<AvgUnit, ?, ?> averageConversion,
      MechanismConversion<DiffUnit, ?, ?> differenceConversion,
      Setpoint<AvgUnit> averageGoal,
      Setpoint<DiffUnit> differenceGoal,
      Supplier<? extends Measure<AvgUnit>> averageTolerance,
      Supplier<? extends Measure<DiffUnit>> differenceTolerance,
      Supplier<? extends Measure<AvgUnit>> averageMeasurement,
      Supplier<? extends Measure<DiffUnit>> differenceMeasurement,
      boolean foc) {
    this(
        core,
        averageConversion,
        differenceConversion,
        averageGoal,
        differenceGoal,
        averageTolerance,
        differenceTolerance,
        averageMeasurement,
        differenceMeasurement,
        foc,
        OutputType.VOLTAGE);
  }

  /** Sets the average goal preserving the current output type. */
  public void setAverageGoal(Setpoint<AvgUnit> goal) {
    this.averageGoal = Objects.requireNonNull(goal, "goal");
    core.activate(this);
  }

  /** Sets the average goal with zero feedforward. */
  public void setAverageGoal(Measure<AvgUnit> goal) {
    averageGoal.setSetpoint(goal);
    core.activate(this);
  }

  /** Sets the average goal with voltage feedforward. */
  public void setAverageGoal(Measure<AvgUnit> goal, Voltage feedforward) {
    this.outputType = OutputType.VOLTAGE;
    this.averageVoltageFeedforward = Objects.requireNonNull(feedforward, "feedforward");
    averageGoal.setSetpoint(goal);
    core.activate(this);
  }

  /** Sets the average goal with current feedforward. */
  public void setAverageGoal(Measure<AvgUnit> goal, Current feedforward) {
    this.outputType = OutputType.CURRENT;
    this.averageCurrentFeedforward = Objects.requireNonNull(feedforward, "feedforward");
    averageGoal.setSetpoint(goal);
    core.activate(this);
  }

  /** Sets the average goal with duty-cycle feedforward. */
  public void setAverageGoal(Measure<AvgUnit> goal, double feedforward) {
    this.outputType = OutputType.DUTY_CYCLE;
    this.averageDutyCycleFeedforward = feedforward;
    averageGoal.setSetpoint(goal);
    core.activate(this);
  }

  /** Sets the difference goal preserving the current output type. */
  public void setDifferenceGoal(Setpoint<DiffUnit> goal) {
    this.differenceGoal = Objects.requireNonNull(goal, "goal");
    core.activate(this);
  }

  /** Sets the difference goal with zero feedforward. */
  public void setDifferenceGoal(Measure<DiffUnit> goal) {
    differenceGoal.setSetpoint(goal);
    core.activate(this);
  }

  /** Sets the difference goal with voltage feedforward. */
  public void setDifferenceGoal(Measure<DiffUnit> goal, Voltage feedforward) {
    this.outputType = OutputType.VOLTAGE;
    this.differenceVoltageFeedforward = Objects.requireNonNull(feedforward, "feedforward");
    differenceGoal.setSetpoint(goal);
    core.activate(this);
  }

  /** Sets the difference goal with current feedforward. */
  public void setDifferenceGoal(Measure<DiffUnit> goal, Current feedforward) {
    this.outputType = OutputType.CURRENT;
    this.differenceCurrentFeedforward = Objects.requireNonNull(feedforward, "feedforward");
    differenceGoal.setSetpoint(goal);
    core.activate(this);
  }

  /** Sets the difference goal with duty-cycle feedforward. */
  public void setDifferenceGoal(Measure<DiffUnit> goal, double feedforward) {
    this.outputType = OutputType.DUTY_CYCLE;
    this.differenceDutyCycleFeedforward = feedforward;
    differenceGoal.setSetpoint(goal);
    core.activate(this);
  }

  /** Sets both average and difference goals. */
  public void setGoals(Setpoint<AvgUnit> average, Setpoint<DiffUnit> difference) {
    this.averageGoal = Objects.requireNonNull(average, "average");
    this.differenceGoal = Objects.requireNonNull(difference, "difference");
    core.activate(this);
  }

  /** Sets both average and difference goals with zero feedforward. */
  public void setGoals(Measure<AvgUnit> average, Measure<DiffUnit> difference) {
    averageGoal.setSetpoint(average);
    differenceGoal.setSetpoint(difference);
    core.activate(this);
  }

  /** Checks if the mechanism is within tolerance of the average goal. */
  public boolean atAverageGoal() {
    Measure<AvgUnit> current = averageMeasurement.get();
    Measure<AvgUnit> target = averageGoal.getNewSetpoint();
    AvgUnit unit = target.unit();
    return Math.abs(current.in(unit) - target.in(unit)) <= averageTolerance.get().in(unit);
  }

  /** Checks if the mechanism is within tolerance of the difference goal. */
  public boolean atDifferenceGoal() {
    Measure<DiffUnit> current = differenceMeasurement.get();
    Measure<DiffUnit> target = differenceGoal.getNewSetpoint();
    DiffUnit unit = target.unit();
    return Math.abs(current.in(unit) - target.in(unit)) <= differenceTolerance.get().in(unit);
  }

  /** Checks if both axes are at their goals. */
  public boolean atGoals() {
    return atAverageGoal() && atDifferenceGoal();
  }

  /** Returns a command that completes once both goals are reached. */
  public Command waitUntilAtGoals() {
    return Commands.waitUntil(this::atGoals);
  }

  @Override
  public String name() {
    return switch (outputType) {
      case VOLTAGE -> "DifferentialVoltage";
      case CURRENT -> "DifferentialCurrent";
      case DUTY_CYCLE -> "DifferentialDutyCycle";
    };
  }

  @Override
  public MotorRequest request() {
    Angle avgPos = averageConversion.toMotorPosition(averageGoal.getNewSetpoint());
    Angle diffPos = differenceConversion.toMotorPosition(differenceGoal.getNewSetpoint());

    AxisRequest averageRequest =
        switch (outputType) {
          case VOLTAGE ->
              switch (profileMode) {
                case PROFILED ->
                    new PositionRequest.ProfiledPositionVoltage(
                        avgPos, averageVoltageFeedforward, foc, averageSlot, options);
                case UNPROFILED ->
                    new PositionRequest.PositionVoltage(
                        avgPos,
                        RotationsPerSecond.of(0.0),
                        averageVoltageFeedforward,
                        foc,
                        averageSlot,
                        options);
                case EXPONENTIAL ->
                    new PositionRequest.ExponentialProfiledPositionVoltage(
                        avgPos, averageVoltageFeedforward, foc, averageSlot, options);
              };
          case CURRENT ->
              switch (profileMode) {
                case PROFILED ->
                    new PositionRequest.ProfiledPositionCurrent(
                        avgPos, averageCurrentFeedforward, averageSlot, options);
                case UNPROFILED ->
                    new PositionRequest.PositionCurrent(
                        avgPos,
                        RotationsPerSecond.of(0.0),
                        averageCurrentFeedforward,
                        averageSlot,
                        options);
                case EXPONENTIAL ->
                    new PositionRequest.ExponentialProfiledPositionCurrent(
                        avgPos, averageCurrentFeedforward, averageSlot, options);
              };
          case DUTY_CYCLE ->
              switch (profileMode) {
                case PROFILED ->
                    new PositionRequest.ProfiledPositionDutyCycle(
                        avgPos, averageDutyCycleFeedforward, foc, averageSlot, options);
                case UNPROFILED ->
                    new PositionRequest.PositionDutyCycle(
                        avgPos,
                        RotationsPerSecond.of(0.0),
                        averageDutyCycleFeedforward,
                        foc,
                        averageSlot,
                        options);
                case EXPONENTIAL ->
                    new PositionRequest.ExponentialProfiledPositionDutyCycle(
                        avgPos, averageDutyCycleFeedforward, foc, averageSlot, options);
              };
        };

    AxisRequest differenceRequest =
        switch (outputType) {
          case VOLTAGE ->
              new PositionRequest.PositionVoltage(
                  diffPos,
                  RotationsPerSecond.of(0.0),
                  differenceVoltageFeedforward,
                  foc,
                  differenceSlot,
                  options);
          case CURRENT ->
              new PositionRequest.PositionCurrent(
                  diffPos,
                  RotationsPerSecond.of(0.0),
                  differenceCurrentFeedforward,
                  differenceSlot,
                  options);
          case DUTY_CYCLE ->
              new PositionRequest.PositionDutyCycle(
                  diffPos,
                  RotationsPerSecond.of(0.0),
                  differenceDutyCycleFeedforward,
                  foc,
                  differenceSlot,
                  options);
        };

    return new Differential(averageRequest, differenceRequest, options.updateFrequency());
  }

  @Override
  public void periodic(String topic) {
    Logger.recordOutputMeasure(topic + "/AverageGoal", averageGoal.getNewSetpoint());
    Logger.recordOutputMeasure(topic + "/DifferenceGoal", differenceGoal.getNewSetpoint());
    Logger.recordOutput(topic + "/DifferentialMode", name());
  }
}
