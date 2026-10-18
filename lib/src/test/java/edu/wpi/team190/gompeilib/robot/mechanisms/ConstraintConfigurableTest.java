package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMechanism;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMotors;
import org.junit.jupiter.api.Test;

public class ConstraintConfigurableTest extends GompeiLibTestBase {
  @Test
  public void testSendsTheProfileBuiltFromTheConstraints() {
    TestMotors motors = new TestMotors();
    TestMechanism mechanism = new TestMechanism(motors);

    mechanism.updateConstraints(
        AngularPositionConstraints.builder()
            .withMaxVelocity(RotationsPerSecond.of(0.5))
            .withMaxAcceleration(RotationsPerSecondPerSecond.of(1.0))
            .build());

    verify(motors.motor)
        .updateMotionProfile(
            MotionProfileConfig.trapezoidal(
                RotationsPerSecond.of(0.5), RotationsPerSecondPerSecond.of(1.0)));
  }
}
