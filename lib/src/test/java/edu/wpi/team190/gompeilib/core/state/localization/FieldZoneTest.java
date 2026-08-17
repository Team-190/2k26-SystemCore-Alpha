package edu.wpi.team190.gompeilib.core.state.localization;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Set;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.vision.apriltag.AprilTag;

public class FieldZoneTest {
  @Test
  public void testFieldZone() {
    AprilTag tag = new AprilTag(1, new Pose3d());
    FieldZone zone = new FieldZone(Set.of(tag));
    assertEquals(Set.of(tag), zone.aprilTags());
  }
}
