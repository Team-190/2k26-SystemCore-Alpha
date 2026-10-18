package edu.wpi.team190.gompeilib.sysid;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Volts;

import edu.wpi.team190.gompeilib.tuning.GompeiTunables;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.system.Timer;
import org.wpilib.tunable.Tunable;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.Unit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Per;

/**
 * Finds a mechanism's static friction: ramps an open-loop output until the mechanism starts moving,
 * then reports the output it took. The output can be any unit the mechanism is controlled in; start
 * from {@link #voltage} or {@link #current}, which ramp at a tunable rate.
 *
 * @param <U> the output unit
 */
public class KSCharacterization<U extends Unit> extends Command {
  private static final Tunable<Per<VoltageUnit, TimeUnit>> voltageRamp =
      GompeiTunables.value("StaticCharacterization/Voltage Ramp", Volts.per(Second).ofNative(0.1));
  private static final Tunable<Per<CurrentUnit, TimeUnit>> currentRamp =
      GompeiTunables.value("StaticCharacterization/Current Ramp", Amps.per(Second).ofNative(1.0));
  private static final Tunable<AngularVelocity> minVelocity =
      GompeiTunables.value(
          "StaticCharacterization/Min Static Velocity", RotationsPerSecond.of(0.1));

  private final U outputUnit;
  private final Tunable<Per<U, TimeUnit>> rampRate;
  private final Consumer<Measure<U>> output;
  private final Supplier<AngularVelocity> velocity;
  private final Timer timer = new Timer();
  private Measure<U> currentOutput;

  /**
   * Ramps an output until the mechanism's velocity passes the minimum static velocity.
   *
   * @param subsystem the subsystem to require
   * @param outputUnit the output unit
   * @param rampRate how fast to ramp the output
   * @param output applies the output to the mechanism
   * @param velocity the mechanism's velocity
   */
  public KSCharacterization(
      Subsystem subsystem,
      U outputUnit,
      Tunable<Per<U, TimeUnit>> rampRate,
      Consumer<Measure<U>> output,
      Supplier<AngularVelocity> velocity) {
    this.outputUnit = outputUnit;
    this.rampRate = rampRate;
    this.output = output;
    this.velocity = velocity;
    currentOutput = output(0.0);
    addRequirements(subsystem);
  }

  /** Ramps a voltage at the tunable {@code StaticCharacterization/Voltage Ramp} rate. */
  public static KSCharacterization<VoltageUnit> voltage(
      Subsystem subsystem,
      Consumer<Measure<VoltageUnit>> output,
      Supplier<AngularVelocity> velocity) {
    return new KSCharacterization<>(subsystem, Volts, voltageRamp, output, velocity);
  }

  /** Ramps a torque current at the tunable {@code StaticCharacterization/Current Ramp} rate. */
  public static KSCharacterization<CurrentUnit> current(
      Subsystem subsystem,
      Consumer<Measure<CurrentUnit>> output,
      Supplier<AngularVelocity> velocity) {
    return new KSCharacterization<>(subsystem, Amps, currentRamp, output, velocity);
  }

  // Unit#ofBaseUnits only returns Measure<?> since U can't be reified here.
  @SuppressWarnings("unchecked")
  private Measure<U> output(double baseUnits) {
    return (Measure<U>) outputUnit.ofBaseUnits(baseUnits);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    currentOutput = output(timer.get() * rampRate.get().baseUnitMagnitude());
    output.accept(currentOutput);
  }

  @Override
  public boolean isFinished() {
    return velocity.get().gte(minVelocity.get());
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("********** FF Characterization Results **********");
    System.out.println("Static Characterization output: " + currentOutput.toLongString());
    Logger.recordOutputMeasure("KS output", currentOutput);
    output.accept(output(0.0));
  }
}
