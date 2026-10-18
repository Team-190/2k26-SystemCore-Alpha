package first.robot.v3_Horse_CV2_TR.commands;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Second;

import edu.wpi.team190.gompeilib.tuning.GompeiTunables;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.system.Timer;
import org.wpilib.tunable.Tunable;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Per;

public class KSCharacterization extends Command {
  private static final Tunable<Per<CurrentUnit, TimeUnit>> currentRamp =
      GompeiTunables.value("StaticCharacterization/Current Ramp", Amps.per(Second).ofNative(1.0));
  private static final Tunable<AngularVelocity> minVelocity =
      GompeiTunables.value(
          "StaticCharacterization/Min Static Velocity", RotationsPerSecond.of(0.1));

  private final DoubleConsumer inputConsumer;
  private final DoubleSupplier velocitySupplier;
  private final Timer timer = new Timer();
  private double currentInput = 0.0;

  /**
   * Ramps the characterization input in amps until the velocity, in rotations per second, passes
   * the minimum.
   */
  public KSCharacterization(
      Subsystem subsystem,
      DoubleConsumer characterizationInputConsumer,
      DoubleSupplier velocitySupplier) {
    inputConsumer = characterizationInputConsumer;
    this.velocitySupplier = velocitySupplier;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    currentInput = timer.get() * currentRamp.get().in(Amps.per(Second));
    inputConsumer.accept(currentInput);
  }

  @Override
  public boolean isFinished() {
    return velocitySupplier.getAsDouble() >= minVelocity.get().in(RotationsPerSecond);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("********** FF Characterization Results **********");
    System.out.println("Static Characterization output: " + currentInput + " amps");
    Logger.recordOutput("KS output", currentInput);
    inputConsumer.accept(0);
  }
}
