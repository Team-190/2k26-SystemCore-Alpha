package edu.wpi.team190.gompeilib.io.motor.interfaces;

import edu.wpi.team190.gompeilib.io.base.ConnectedIO;
import edu.wpi.team190.gompeilib.io.base.CurrentLimitConfigurableIO;
import edu.wpi.team190.gompeilib.io.base.GainSlotControllableIO;
import edu.wpi.team190.gompeilib.io.base.MotionProfileConfigurableIO;
import edu.wpi.team190.gompeilib.io.base.PositionResettableIO;
import edu.wpi.team190.gompeilib.io.base.RequestControllableIO;
import edu.wpi.team190.gompeilib.io.base.StickyFaultClearableIO;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;

/**
 * One physical motor controller. Implementations run every {@link MotorRequest} the device supports
 * and throw {@link UnsupportedOperationException} for the rest; subsystems decide which requests to
 * send.
 */
public interface MotorIO
    extends ConnectedIO,
        RequestControllableIO<MotorRequest>,
        PositionResettableIO,
        GainSlotControllableIO,
        MotionProfileConfigurableIO,
        CurrentLimitConfigurableIO,
        StickyFaultClearableIO {

  /**
   * Fills {@code telemetry} with the motor's latest readings.
   *
   * @param telemetry the telemetry to fill
   */
  default void updateInputs(MotorTelemetry telemetry) {}

  /**
   * Returns a motor for log replay: telemetry comes from the log, so it reads nothing and applies
   * no requests.
   *
   * @return the replay motor
   */
  static MotorIO replay() {
    return request -> {};
  }

  /** Builds {@link #replay()} motors. */
  MotorIOFactory REPLAY_FACTORY =
      new MotorIOFactory() {
        @Override
        public MotorIO create(MotorConfig config) {
          return replay();
        }

        @Override
        public String family() {
          return "Replay";
        }
      };
}
