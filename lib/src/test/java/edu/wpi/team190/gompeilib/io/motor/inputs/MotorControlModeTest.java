package edu.wpi.team190.gompeilib.io.motor.inputs;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class MotorControlModeTest {

  @Test
  public void testConstants() {
    IOTestSupport.assertEnumRoundTrip(
        MotorControlMode.class,
        "UNKNOWN",
        "DISABLED",
        "NEUTRAL",
        "STATIC_BRAKE",
        "COAST",
        "FOLLOWER",
        "TONE",
        "UNAUTHORIZED",
        "DUTY_CYCLE_OUTPUT",
        "POSITION_DUTY_CYCLE",
        "VELOCITY_DUTY_CYCLE",
        "PROFILED_POSITION_DUTY_CYCLE",
        "PROFILED_VELOCITY_DUTY_CYCLE",
        "EXPONENTIAL_PROFILED_POSITION_DUTY_CYCLE",
        "VOLTAGE_OUTPUT",
        "POSITION_VOLTAGE",
        "VELOCITY_VOLTAGE",
        "PROFILED_POSITION_VOLTAGE",
        "PROFILED_VELOCITY_VOLTAGE",
        "EXPONENTIAL_PROFILED_POSITION_VOLTAGE",
        "CURRENT_OUTPUT",
        "POSITION_CURRENT",
        "VELOCITY_CURRENT",
        "PROFILED_POSITION_CURRENT",
        "PROFILED_VELOCITY_CURRENT",
        "EXPONENTIAL_PROFILED_POSITION_CURRENT");
  }
}
