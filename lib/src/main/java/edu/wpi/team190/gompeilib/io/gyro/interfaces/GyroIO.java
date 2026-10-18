package edu.wpi.team190.gompeilib.io.gyro.interfaces;

import edu.wpi.team190.gompeilib.io.base.ConnectedIO;
import edu.wpi.team190.gompeilib.io.base.StickyFaultClearableIO;
import edu.wpi.team190.gompeilib.io.base.YawResettableIO;
import edu.wpi.team190.gompeilib.io.gyro.inputs.GyroTelemetry;

/** One gyro. */
public interface GyroIO extends ConnectedIO, YawResettableIO, StickyFaultClearableIO {
  /**
   * Fills {@code telemetry} with the gyro's latest readings, including any yaw samples captured for
   * odometry since the last call.
   *
   * @param telemetry the telemetry to fill
   */
  default void updateInputs(GyroTelemetry telemetry) {}
}
