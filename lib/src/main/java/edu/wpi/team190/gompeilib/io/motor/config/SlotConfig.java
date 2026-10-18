package edu.wpi.team190.gompeilib.io.motor.config;

import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import java.util.Objects;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.Unit;

/**
 * The closed-loop gains in one slot, at the gearbox output. A slot holds either position gains (per
 * angle) or velocity gains (per angular velocity), and its gains' output unit is duty cycle, volts,
 * or amps, which must match the output type of the requests that use the slot.
 */
public sealed interface SlotConfig {
  /**
   * Returns how {@code kG} is applied.
   *
   * @return the gravity type
   */
  GravityType gravityType();

  /**
   * Returns the gains.
   *
   * @return the gains
   */
  Gains<?, ?, AngularVelocityUnit> gains();

  /**
   * Returns the output type the gains are in.
   *
   * @return the output type
   */
  default OutputType outputType() {
    return motorOutputType(gains().outputUnit());
  }

  /**
   * A slot for position requests.
   *
   * @param gains the gains, per gearbox-output angle
   * @param gravityType how {@code kG} is applied
   */
  record Position(Gains<?, AngleUnit, AngularVelocityUnit> gains, GravityType gravityType)
      implements SlotConfig {
    public Position {
      Objects.requireNonNull(gains, "gains");
      Objects.requireNonNull(gravityType, "gravityType");
      motorOutputType(gains.outputUnit());
    }
  }

  /**
   * A slot for velocity requests.
   *
   * @param gains the gains, per gearbox-output angular velocity
   * @param gravityType how {@code kG} is applied
   */
  record Velocity(Gains<?, AngularVelocityUnit, AngularVelocityUnit> gains, GravityType gravityType)
      implements SlotConfig {
    public Velocity {
      Objects.requireNonNull(gains, "gains");
      Objects.requireNonNull(gravityType, "gravityType");
      motorOutputType(gains.outputUnit());
    }
  }

  /**
   * Creates a position slot whose {@code kG} is a constant.
   *
   * @param gains the gains
   * @return the slot
   */
  static Position position(Gains<?, AngleUnit, AngularVelocityUnit> gains) {
    return new Position(gains, GravityType.CONSTANT);
  }

  /**
   * Creates a position slot.
   *
   * @param gains the gains
   * @param gravityType how {@code kG} is applied
   * @return the slot
   */
  static Position position(
      Gains<?, AngleUnit, AngularVelocityUnit> gains, GravityType gravityType) {
    return new Position(gains, gravityType);
  }

  /**
   * Creates a velocity slot whose {@code kG} is a constant.
   *
   * @param gains the gains
   * @return the slot
   */
  static Velocity velocity(Gains<?, AngularVelocityUnit, AngularVelocityUnit> gains) {
    return new Velocity(gains, GravityType.CONSTANT);
  }

  /**
   * Checks that a slot's gains are in the output type its requests use.
   *
   * @param slot the slot
   * @param expected the output type the requests use
   * @param name what the slot is, for the error message
   * @throws IllegalArgumentException if the gains are in another output type
   */
  static void requireOutputType(SlotConfig slot, OutputType expected, String name) {
    if (slot.outputType() != expected) {
      throw new IllegalArgumentException(
          name
              + " gains are in "
              + slot.gains().outputUnit()
              + " ("
              + slot.outputType()
              + ") but its requests output "
              + expected);
    }
  }

  private static OutputType motorOutputType(Unit unit) {
    return OutputType.of(unit)
        .orElseThrow(
            () ->
                new IllegalArgumentException(
                    "Slot gains must output duty cycle, volts, or amps, not " + unit));
  }
}
