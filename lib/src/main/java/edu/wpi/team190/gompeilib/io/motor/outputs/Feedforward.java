package edu.wpi.team190.gompeilib.io.motor.outputs;

import java.util.Objects;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

/**
 * A closed-loop feedforward. Its kind is the output the loop drives, so passing one picks the
 * output type: a voltage feedforward runs the voltage loop, a torque-current one the current loop,
 * a duty cycle one the duty-cycle loop.
 */
public sealed interface Feedforward {
  /**
   * Returns the output type this feedforward drives.
   *
   * @return the output type
   */
  OutputType type();

  /**
   * A voltage feedforward.
   *
   * @param voltage the feedforward
   */
  record Volts(Voltage voltage) implements Feedforward {
    public Volts {
      Objects.requireNonNull(voltage, "voltage");
    }

    @Override
    public OutputType type() {
      return OutputType.VOLTAGE;
    }
  }

  /**
   * A torque-current feedforward.
   *
   * @param current the feedforward
   */
  record Amps(Current current) implements Feedforward {
    public Amps {
      Objects.requireNonNull(current, "current");
    }

    @Override
    public OutputType type() {
      return OutputType.CURRENT;
    }
  }

  /**
   * A duty-cycle feedforward.
   *
   * @param dutyCycle the feedforward in [-1, 1]
   */
  record DutyCycle(double dutyCycle) implements Feedforward {
    @Override
    public OutputType type() {
      return OutputType.DUTY_CYCLE;
    }
  }

  static Feedforward volts(Voltage voltage) {
    return new Volts(voltage);
  }

  static Feedforward amps(Current current) {
    return new Amps(current);
  }

  static Feedforward dutyCycle(double dutyCycle) {
    return new DutyCycle(dutyCycle);
  }

  /**
   * Returns a zero feedforward of an output type.
   *
   * @param type the output type
   * @return the feedforward
   */
  static Feedforward zero(OutputType type) {
    return switch (Objects.requireNonNull(type, "type")) {
      case VOLTAGE -> new Volts(org.wpilib.units.Units.Volts.of(0.0));
      case CURRENT -> new Amps(org.wpilib.units.Units.Amps.of(0.0));
      case DUTY_CYCLE -> new DutyCycle(0.0);
    };
  }
}
