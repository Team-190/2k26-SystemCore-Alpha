package edu.wpi.team190.gompeilib.robot.subsystems.vision.camera;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class CameraTypeTest {

  @Test
  public void testEnumValues() {
    assertEquals(0.067, CameraType.BLINK_TIME);

    for (CameraType type : CameraType.values()) {
      assertNotNull(type.name());
      assertNotNull(CameraType.valueOf(type.name()));

      // Basic assertions depending on type
      switch (type) {
        case LIMELIGHT_2_PLUS:
          assertTrue(type.horizontalFOV > 0);
          assertTrue(type.verticalFOV > 0);
          assertTrue(type.megatagXY > 0);
          assertTrue(type.megatag2XY > 0);
          assertTrue(type.theta > 0);
          break;
        case LIMELIGHT_3:
          assertTrue(type.horizontalFOV > 0);
          assertTrue(type.verticalFOV > 0);
          assertTrue(type.megatagXY > 0);
          assertTrue(type.megatag2XY > 0);
          assertTrue(type.theta > 0);
          break;
        case LIMELIGHT_3G:
          assertTrue(type.horizontalFOV > 0);
          assertTrue(type.verticalFOV > 0);
          assertTrue(type.megatagXY > 0);
          assertTrue(type.megatag2XY > 0);
          assertTrue(type.theta > 0);
          break;
        case LIMELIGHT_4:
          assertTrue(type.horizontalFOV > 0);
          assertTrue(type.verticalFOV > 0);
          assertTrue(type.megatagXY > 0);
          assertTrue(type.megatag2XY > 0);
          assertTrue(type.theta > 0);
          break;
        case THRIFTYCAM:
          assertTrue(type.horizontalFOV > 0);
          assertTrue(type.verticalFOV > 0);
          assertTrue(type.megatagXY > 0);
          assertTrue(type.megatag2XY > 0);
          assertTrue(type.theta > 0);
          break;
        case DEFAULT:
          assertEquals(0.0, type.horizontalFOV);
          assertEquals(0.0, type.verticalFOV);
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
