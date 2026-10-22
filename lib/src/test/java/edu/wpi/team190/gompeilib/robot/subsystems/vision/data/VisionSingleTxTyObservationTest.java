package edu.wpi.team190.gompeilib.robot.subsystems.vision.data;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;

public class VisionSingleTxTyObservationTest {

  @Test
  public void testRecord() {
    Pose3d cameraPose = new Pose3d();
    VisionSingleTxTyObservation obs =
        new VisionSingleTxTyObservation(15, 1.1, 2.2, 3.3, 4.4, cameraPose);

    assertEquals(15, obs.tagId());
    assertEquals(Degrees.of(1.1), obs.tx());
    assertEquals(Degrees.of(2.2), obs.ty());
    assertEquals(Meters.of(3.3), obs.distance());
    assertEquals(Seconds.of(4.4), obs.timestamp());
    assertEquals(cameraPose, obs.cameraPose());

    VisionSingleTxTyObservation measureObs =
        new VisionSingleTxTyObservation(
            15, Degrees.of(1.1), Degrees.of(2.2), Meters.of(3.3), Seconds.of(4.4), cameraPose);
    assertEquals(obs, measureObs);

    assertNotNull(obs.toString());
    assertEquals(obs.hashCode(), obs.hashCode());
    assertEquals(obs, obs);
  }
}
