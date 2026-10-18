package edu.wpi.team190.gompeilib.robot.mechanisms;

import org.wpilib.units.Measure;
import org.wpilib.units.Unit;

/**
 * A mechanism whose position and velocity can be read in its own units.
 *
 * @param <P> the mechanism's position unit
 * @param <V> the mechanism's velocity unit
 */
public interface MotionReadable<P extends Unit, V extends Unit> extends Mechanism {
  /**
   * Returns the mechanism's conversion to the motors' gearbox output.
   *
   * @return the conversion
   */
  MechanismConversion<P, V, ?> conversion();

  default Measure<P> getPosition() {
    return conversion().fromMotorPosition(motors().leader().position);
  }

  default Measure<V> getVelocity() {
    return conversion().fromMotorVelocity(motors().leader().velocity);
  }

  /**
   * Returns the velocity the active profile is commanding, in the mechanism's unit.
   *
   * @return the profiled velocity
   */
  default Measure<V> getVelocitySetpoint() {
    return conversion().fromMotorVelocity(motors().leader().closedLoopPositionReferenceSlope);
  }
}
