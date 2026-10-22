package edu.wpi.team190.gompeilib.robot.subsystems.vision.camera;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Seconds;

import org.junit.jupiter.api.Test;

public class CameraTypeTest {

  @Test
  public void testEnumValues() {
    assertEquals(Seconds.of(0.067), CameraType.BLINK_TIME);

    for (CameraType type : CameraType.values()) {
      assertNotNull(type.name());
      assertNotNull(CameraType.valueOf(type.name()));
      if (type == CameraType.LIMELIGHT_4) {
        assertTrue(type.hasIMU());
      } else {
        assertFalse(type.hasIMU());
      }

      // Basic assertions depending on type
      switch (type) {
        case LIMELIGHT_2_PLUS:
          assertTrue(type.horizontalFOV.gt(Degrees.of(0)));
          assertTrue(type.verticalFOV.gt(Degrees.of(0)));
          assertTrue(type.megatagXY > 0);
          assertTrue(type.megatag2XY > 0);
          assertTrue(type.theta > 0);
          break;
        case LIMELIGHT_3:
          assertTrue(type.horizontalFOV.gt(Degrees.of(0)));
          assertTrue(type.verticalFOV.gt(Degrees.of(0)));
          assertTrue(type.megatagXY > 0);
          assertTrue(type.megatag2XY > 0);
          assertTrue(type.theta > 0);
          break;
        case LIMELIGHT_3G:
          assertTrue(type.horizontalFOV.gt(Degrees.of(0)));
          assertTrue(type.verticalFOV.gt(Degrees.of(0)));
          assertTrue(type.megatagXY > 0);
          assertTrue(type.megatag2XY > 0);
          assertTrue(type.theta > 0);
          break;
        case LIMELIGHT_4:
          assertTrue(type.horizontalFOV.gt(Degrees.of(0)));
          assertTrue(type.verticalFOV.gt(Degrees.of(0)));
          assertTrue(type.megatagXY > 0);
          assertTrue(type.megatag2XY > 0);
          assertTrue(type.theta > 0);
          break;
        case THRIFTYCAM:
          assertTrue(type.horizontalFOV.gt(Degrees.of(0)));
          assertTrue(type.verticalFOV.gt(Degrees.of(0)));
          assertTrue(type.megatagXY > 0);
          assertTrue(type.megatag2XY > 0);
          assertTrue(type.theta > 0);
          break;
        case DEFAULT:
          assertEquals(Degrees.of(0.0), type.horizontalFOV);
          assertEquals(Degrees.of(0.0), type.verticalFOV);
          assertEquals(0.0, type.megatagXY);
          assertEquals(0.0, type.megatag2XY);
          assertEquals(0.0, type.theta);
          assertEquals(0.0, type.primaryXYStandardDeviationCoefficient);
          assertEquals(0.0, type.secondaryXYStandardDeviationCoefficient);
          assertEquals(0.0, type.primaryThetaStandardDeviationCoefficient);
          break;
      }
    }
  }
}
