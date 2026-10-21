package edu.wpi.team190.gompeilib.robot.mechanisms.flywheel;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Value;
import static org.wpilib.units.Units.Volts;

import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.physics.PlantCharacterization;
import lombok.Builder;
import lombok.NonNull;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Dimensionless;
import org.wpilib.units.measure.Voltage;

@Builder(setterPrefix = "with", toBuilder = true)
public class GenericFlywheelConstants {
  /**
   * The leader motor and its followers. Leave its slots empty: the flywheel fills slot {@code ZERO}
   * from {@link #voltageGains} and slot {@code ONE} from {@link #currentGains}.
   */
  @NonNull public final MotorConfig motor;

  /** The voltage velocity gains, at the gearbox output. Unset gains are zero. */
  @NonNull @Builder.Default
  public final Gains<VoltageUnit, AngularVelocityUnit, AngularVelocityUnit> voltageGains =
      Gains.angularVelocity(Volts).build();

  /** The torque-current velocity gains, at the gearbox output. Unset gains are zero. */
  @NonNull @Builder.Default
  public final Gains<CurrentUnit, AngularVelocityUnit, AngularVelocityUnit> currentGains =
      Gains.angularVelocity(Amps).build();

  /** The motors driving the flywheel, for simulation. */
  @NonNull public final DCMotor flywheelGearbox;

  /**
   * How the flywheel responds to its motors, for simulation: what {@link
   * GenericFlywheel#characterizeVoltage()} or {@link GenericFlywheel#characterizeTorque()}
   * measured, or {@link PlantCharacterization#ofInertia} before the flywheel exists.
   */
  @NonNull public final PlantCharacterization<?> plant;

  /** The goal tolerance, and the profile limits for profiled velocity requests. */
  @NonNull public final AngularVelocityConstraints constraints;

  /** How far one increment or decrement moves the velocity goal. */
  @NonNull public final AngularVelocity velocityOffsetStep;

  /** How far one increment or decrement moves the voltage goal. */
  @NonNull public final Voltage voltageOffsetStep;

  /** How far one increment or decrement moves the open-loop torque-current goal. */
  @NonNull @Builder.Default public final Current currentOffsetStep = Amps.of(1.0);

  /** How far one increment or decrement moves the open-loop duty-cycle goal. */
  @NonNull @Builder.Default public final Dimensionless dutyCycleOffsetStep = Value.of(0.05);

  /** How the flywheel is characterized in voltage. */
  @NonNull public final CharacterizationConfig<VoltageUnit> voltageCharacterization;

  /** How the flywheel is characterized in torque current. */
  @NonNull public final CharacterizationConfig<CurrentUnit> currentCharacterization;
}
