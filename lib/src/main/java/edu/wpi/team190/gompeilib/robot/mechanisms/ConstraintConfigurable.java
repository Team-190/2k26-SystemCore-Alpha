package edu.wpi.team190.gompeilib.robot.mechanisms;

import edu.wpi.team190.gompeilib.control.constraints.Constraints;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;

/**
 * A mechanism whose profile limits come from a constraints object.
 *
 * @param <C> the constraints type
 */
public interface ConstraintConfigurable<C extends Constraints<C>> extends Mechanism {
  /**
   * Builds the motors' motion profile from the mechanism's constraints.
   *
   * @param constraints the constraints
   * @return the profile, at the gearbox output
   */
  MotionProfileConfig motionProfile(C constraints);

  default void updateConstraints(C constraints) {
    motors().updateMotionProfile(motionProfile(constraints));
  }
}
