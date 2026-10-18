package edu.wpi.team190.gompeilib.io.motor.outputs;

import static org.wpilib.units.Units.Amps;

import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

/** A request that drives the output directly, without a closed loop. */
public sealed interface OpenLoopRequest extends AxisRequest {

  /**
   * Drives the output with a duty cycle.
   *
   * @param output the duty cycle, in [-1, 1]
   * @param foc whether to use field-oriented control
   * @param options the shared request options
   */
  record DutyCycleOutput(double output, boolean foc, RequestOptions options)
      implements OpenLoopRequest {
    public DutyCycleOutput(double output) {
      this(output, true, RequestOptions.DEFAULT);
    }

    @Override
    public OutputType outputType() {
      return OutputType.DUTY_CYCLE;
    }
  }

  /**
   * Drives the output with a voltage.
   *
   * @param output the voltage
   * @param foc whether to use field-oriented control
   * @param options the shared request options
   */
  record VoltageOutput(Voltage output, boolean foc, RequestOptions options)
      implements OpenLoopRequest {
    public VoltageOutput(Voltage output) {
      this(output, true, RequestOptions.DEFAULT);
    }

    @Override
    public OutputType outputType() {
      return OutputType.VOLTAGE;
    }
  }

  /**
   * Drives the output with a torque-producing current. Always uses field-oriented control.
   *
   * @param output the current
   * @param maxAbsDutyCycle the maximum absolute duty cycle the current loop may apply, in [0, 1]
   * @param deadband currents within this band of zero produce neutral output
   * @param options the shared request options
   */
  record CurrentOutput(
      Current output, double maxAbsDutyCycle, Current deadband, RequestOptions options)
      implements OpenLoopRequest {
    public CurrentOutput(Current output) {
      this(output, 1.0, Amps.of(0.0), RequestOptions.DEFAULT);
    }

    @Override
    public OutputType outputType() {
      return OutputType.CURRENT;
    }
  }
}
