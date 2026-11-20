package edu.wpi.team190.gompeilib.io.motor.outputs;

/** How a closed-loop velocity request reaches its goal. */
public enum VelocityProfileMode {
  /** Trapezoidal acceleration limiting (Motion Magic Velocity). */
  PROFILED,
  /** Unprofiled closed-loop velocity control. */
  UNPROFILED
}
