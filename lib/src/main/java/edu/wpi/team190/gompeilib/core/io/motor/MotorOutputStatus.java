package edu.wpi.team190.gompeilib.core.io.motor;

/** What the motor's output stage is currently doing. */
public enum MotorOutputStatus {
  UNKNOWN,
  OFF,
  STATIC_BRAKING,
  MOTORING,
  DISCORDANT_MOTORING,
  REGEN_BRAKING;
}
