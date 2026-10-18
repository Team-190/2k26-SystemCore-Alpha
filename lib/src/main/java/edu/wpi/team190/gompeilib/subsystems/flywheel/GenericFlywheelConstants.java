package edu.wpi.team190.gompeilib.subsystems.flywheel;

import edu.wpi.team190.gompeilib.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import lombok.Builder;
import lombok.NonNull;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.units.measure.Voltage;

@Builder(setterPrefix = "with")
public class GenericFlywheelConstants {
  /**
   * The leader motor and its followers. Slot {@code ZERO} holds the voltage velocity gains and slot
   * {@code ONE} the torque-current velocity gains.
   */
  @NonNull public final MotorConfig motor;

  /** The motors driving the flywheel, for simulation. */
  @NonNull public final DCMotor flywheelGearbox;

  /** The flywheel's moment of inertia, for simulation. */
  @NonNull public final MomentOfInertia momentOfInertia;

  /** The goal tolerance, and the profile limits for profiled velocity requests. */
  @NonNull public final AngularVelocityConstraints constraints;

  /** How far one increment or decrement moves the velocity goal. */
  @NonNull public final AngularVelocity velocityOffsetStep;

  /** How far one increment or decrement moves the voltage goal. */
  @NonNull public final Voltage voltageOffsetStep;
}
