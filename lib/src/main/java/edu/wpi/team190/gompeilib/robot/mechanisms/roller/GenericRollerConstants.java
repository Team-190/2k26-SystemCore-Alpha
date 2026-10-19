package edu.wpi.team190.gompeilib.robot.mechanisms.roller;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Value;

import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import lombok.Builder;
import lombok.NonNull;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Dimensionless;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.units.measure.Voltage;

@Builder(setterPrefix = "with", toBuilder = true)
public class GenericRollerConstants {
  /** The leader motor and its followers. */
  @NonNull public final MotorConfig motor;

  /** The motors driving the roller, for simulation. */
  @NonNull public final DCMotor rollerGearbox;

  /** The roller's moment of inertia, for simulation. */
  @NonNull public final MomentOfInertia momentOfInertia;

  /** How far one increment or decrement moves the voltage goal. */
  @NonNull public final Voltage voltageOffsetStep;

  /** How far one increment or decrement moves the open-loop current goal. */
  @NonNull @Builder.Default public final Current currentOffsetStep = Amps.of(1.0);

  /** How far one increment or decrement moves the open-loop duty-cycle goal. */
  @NonNull @Builder.Default public final Dimensionless dutyCycleOffsetStep = Value.of(0.05);
}
