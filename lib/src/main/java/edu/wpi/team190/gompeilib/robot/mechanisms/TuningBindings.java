package edu.wpi.team190.gompeilib.robot.mechanisms;

import edu.wpi.team190.gompeilib.control.constraints.Constraints;
import edu.wpi.team190.gompeilib.io.motor.MotorControllerGroup;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import java.util.function.Function;

/** Forwards tunable gain and constraint changes to a mechanism's motors. */
public final class TuningBindings {
  private TuningBindings() {}

  /**
   * Sends every configured slot's gains to the motors whenever they change.
   *
   * @param motors the mechanism's motors
   * @param config the leader's config, whose slots are watched
   */
  public static void bindGains(MotorControllerGroup motors, MotorConfig config) {
    config.slots.forEach(
        (slot, slotConfig) ->
            slotConfig.gains().onChange(gains -> motors.updateGains(slotConfig, slot)));
  }

  /**
   * Sends the motion profile built from the constraints to the motors whenever they change.
   *
   * @param motors the mechanism's motors
   * @param constraints the mechanism's constraints
   * @param profile how the constraints become a motion profile
   * @param <C> the constraints type
   */
  public static <C extends Constraints<C>> void bindConstraints(
      MotorControllerGroup motors, C constraints, Function<C, MotionProfileConfig> profile) {
    constraints.onChange(changed -> motors.updateMotionProfile(profile.apply(changed)));
  }
}
