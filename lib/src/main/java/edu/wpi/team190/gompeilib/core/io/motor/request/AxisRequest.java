package edu.wpi.team190.gompeilib.core.io.motor.request;

/** A request that drives a single axis of output, open or closed loop. */
public sealed interface AxisRequest extends MotorRequest
    permits OpenLoopRequest, ClosedLoopRequest {
  /**
   * Returns the quantity this request drives the output with.
   *
   * @return the output type
   */
  OutputType outputType();

  /**
   * Returns the shared request options.
   *
   * @return the options
   */
  RequestOptions options();
}
