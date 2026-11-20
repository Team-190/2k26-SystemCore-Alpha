package edu.wpi.team190.gompeilib.robot.common.encoder;

import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismTestSupport.TestMechanism;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismTestSupport.TestMotors;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;

public class EncoderSeedableTest extends GompeiLibTestBase {
  @Test
  public void testSeedsThroughTheEncoderSeed() {
    try (MockedStatic<Logger> logger = mockStatic(Logger.class)) {
      TestMotors motors = new TestMotors();
      TestMechanism mechanism = new TestMechanism(motors);

      mechanism.seedPositionFromEncoder();
      verify(motors.motor).setPosition(Rotations.of(0.25));
    }
  }
}
