package edu.wpi.team190.gompeilib.io.motor.outputs;

/** How a closed-loop request reaches its goal. */
public enum ProfileMode {
  /** Trapezoidal motion profiling (Motion Magic). */
  PROFILED,
  /** Unprofiled closed-loop control. */
  UNPROFILED,
  /** Exponential motion profiling (Motion Magic Expo). Position only. */
  EXPONENTIAL
}
