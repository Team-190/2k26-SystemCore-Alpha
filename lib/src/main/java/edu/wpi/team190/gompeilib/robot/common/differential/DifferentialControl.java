package edu.wpi.team190.gompeilib.robot.common.differential;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.outputs.Differential;
import edu.wpi.team190.gompeilib.io.motor.outputs.Feedforward;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.motor.outputs.PositionRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.ProfileMode;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismConversion;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismCore;
import java.util.Objects;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;

/**
 * Closed-loop differential control for mechanisms driven by two coupled motors on average and
 * difference axes. Both axes drive the same output type, which their feedforwards' kind picks: a
 * feedforward of a new kind on one axis zeroes the other axis's feedforward in that kind.
 *
 * @param <AvgUnit> the average axis unit
 * @param <DiffUnit> the difference axis unit
 */
public final class DifferentialControl<AvgUnit extends Unit, DiffUnit extends Unit>
    implements MechanismCore.Control {
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
  @Getter private Feedforward averageFeedforward;
  @Getter private Feedforward differenceFeedforward;
  @Getter @Setter private GainSlot averageSlot = GainSlot.ZERO;
  @Getter @Setter private GainSlot differenceSlot = GainSlot.ONE;
  @Getter private ProfileMode profileMode = ProfileMode.PROFILED;
  @Getter private RequestOptions options = RequestOptions.DEFAULT;

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
    setOutputType(outputType);
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

  /**
   * Returns the output both axes drive.
   *
   * @return the output type
   */
  public OutputType getOutputType() {
    return averageFeedforward.type();
  }

  /**
   * Switches both axes to an output type, with zero feedforward.
   *
   * @param outputType the output type
   */
  public void setOutputType(OutputType outputType) {
    averageFeedforward = Feedforward.zero(outputType);
    differenceFeedforward = Feedforward.zero(outputType);
  }

  public void setProfileMode(ProfileMode profileMode) {
    this.profileMode = Objects.requireNonNull(profileMode, "profileMode");
  }

  public void setOptions(RequestOptions options) {
    this.options = Objects.requireNonNull(options, "options");
  }

  /** Sets the average goal preserving the current output type. */
  public void setAverageGoal(Setpoint<AvgUnit> goal) {
    this.averageGoal = Objects.requireNonNull(goal, "goal");
    core.activate(this);
  }

  /** Sets the average goal preserving the current feedforward and output type. */
  public void setAverageGoal(Measure<AvgUnit> goal) {
    averageGoal.setSetpoint(goal);
    core.activate(this);
  }

  /** Sets the average goal and feedforward, whose kind picks the output type of both axes. */
  public void setAverageGoal(Measure<AvgUnit> goal, Feedforward feedforward) {
    averageFeedforward = Objects.requireNonNull(feedforward, "feedforward");
    if (differenceFeedforward.type() != feedforward.type()) {
      differenceFeedforward = Feedforward.zero(feedforward.type());
    }
    setAverageGoal(goal);
  }

  /** Sets the difference goal preserving the current output type. */
  public void setDifferenceGoal(Setpoint<DiffUnit> goal) {
    this.differenceGoal = Objects.requireNonNull(goal, "goal");
    core.activate(this);
  }

  /** Sets the difference goal preserving the current feedforward and output type. */
  public void setDifferenceGoal(Measure<DiffUnit> goal) {
    differenceGoal.setSetpoint(goal);
    core.activate(this);
  }

  /** Sets the difference goal and feedforward, whose kind picks the output type of both axes. */
  public void setDifferenceGoal(Measure<DiffUnit> goal, Feedforward feedforward) {
    differenceFeedforward = Objects.requireNonNull(feedforward, "feedforward");
    if (averageFeedforward.type() != feedforward.type()) {
      averageFeedforward = Feedforward.zero(feedforward.type());
    }
    setDifferenceGoal(goal);
  }

  /** Sets both average and difference goals. */
  public void setGoals(Setpoint<AvgUnit> average, Setpoint<DiffUnit> difference) {
    this.averageGoal = Objects.requireNonNull(average, "average");
    this.differenceGoal = Objects.requireNonNull(difference, "difference");
    core.activate(this);
  }

  /** Sets both average and difference goals, keeping the feedforwards. */
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
    return switch (getOutputType()) {
      case VOLTAGE -> "DifferentialVoltage";
      case CURRENT -> "DifferentialCurrent";
      case DUTY_CYCLE -> "DifferentialDutyCycle";
    };
  }

  @Override
  public MotorRequest request() {
    return new Differential(
        PositionRequest.of(
            profileMode,
            averageConversion.toMotorPosition(averageGoal.getNewSetpoint()),
            averageFeedforward,
            foc,
            averageSlot,
            options),
        PositionRequest.of(
            ProfileMode.UNPROFILED,
            differenceConversion.toMotorPosition(differenceGoal.getNewSetpoint()),
            differenceFeedforward,
            foc,
            differenceSlot,
            options),
        options.updateFrequency());
  }

  @Override
  public void periodic(String topic) {
    Logger.recordOutputMeasure(topic + "/AverageGoal", averageGoal.getNewSetpoint());
    Logger.recordOutputMeasure(topic + "/DifferenceGoal", differenceGoal.getNewSetpoint());
    Logger.recordOutput(topic + "/DifferentialMode", name());
  }
}
