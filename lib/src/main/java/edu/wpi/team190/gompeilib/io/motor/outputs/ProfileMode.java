package edu.wpi.team190.gompeilib.io.motor.outputs;

/**
 * How a closed-loop position request reaches its goal. Velocity uses {@link VelocityProfileMode}.
 */
public enum ProfileMode {
  /** Trapezoidal motion profiling (Motion Magic). */
  PROFILED,
  /** Unprofiled closed-loop control. */
  UNPROFILED,
  /** Exponential motion profiling (Motion Magic Expo). */
  EXPONENTIAL
}
