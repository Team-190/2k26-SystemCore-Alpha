package edu.wpi.team190.gompeilib.io.motor.config;

/** How a slot's gravity gain {@code kG} is applied. */
public enum GravityType {
  /** {@code kG} is a constant, as for an elevator. */
  CONSTANT,
  /** {@code kG} is scaled by the cosine of the position, as for an arm. */
  COSINE;
}
