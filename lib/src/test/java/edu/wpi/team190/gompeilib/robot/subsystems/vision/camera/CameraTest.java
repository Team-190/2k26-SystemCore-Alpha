package edu.wpi.team190.gompeilib.robot.subsystems.vision.camera;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.robot.subsystems.vision.data.VisionMultiTxTyObservation;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.data.VisionPoseObservation;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.data.VisionSingleTxTyObservation;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.Field;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.linalg.VecBuilder;

public class CameraTest {

  private static class TestCamera extends Camera {
    public TestCamera(
        String name,
        List<Consumer<List<VisionPoseObservation>>> poseObservers,
        List<Consumer<List<VisionMultiTxTyObservation>>> multiTxTyObservers,
        List<Consumer<List<VisionSingleTxTyObservation>>> singleTxTyObservers) {
      super(name, poseObservers, multiTxTyObservers, singleTxTyObservers);
    }
  }

  @Test
  public void testCameraBaseMethods() {
    List<List<VisionPoseObservation>> receivedPose = new ArrayList<>();
    List<List<VisionMultiTxTyObservation>> receivedMulti = new ArrayList<>();
    List<List<VisionSingleTxTyObservation>> receivedSingle = new ArrayList<>();

    Consumer<List<VisionPoseObservation>> poseObserver = receivedPose::add;
    Consumer<List<VisionMultiTxTyObservation>> multiObserver = receivedMulti::add;
    Consumer<List<VisionSingleTxTyObservation>> singleObserver = receivedSingle::add;

    TestCamera camera =
        new TestCamera(
            "TestCam", List.of(poseObserver), List.of(multiObserver), List.of(singleObserver));

    assertEquals("TestCam", camera.getName());
    assertNotNull(camera.getPoseObservationList());
    assertNotNull(camera.getMultiTxTyObservationList());
    assertNotNull(camera.getSingleTxTyObservationList());

    Pose3d pose = new Pose3d();
    camera.setCameraPose(pose);
    assertEquals(pose, camera.getCurrentCameraPose());

    // Call periodic (no-op in base class)
    camera.periodic();

    // Trigger observers when lists are empty
    camera.sendObservers();
    assertEquals(1, receivedPose.size());
    assertEquals(1, receivedMulti.size());
    assertEquals(1, receivedSingle.size());
    assertTrue(receivedPose.get(0).isEmpty());
    assertTrue(receivedMulti.get(0).isEmpty());
    assertTrue(receivedSingle.get(0).isEmpty());

    // Verify observers receive an immutable copy (mutating throws UnsupportedOperationException)
    camera
        .getPoseObservationList()
        .add(
            new VisionPoseObservation(
                new Pose2d(), java.util.Set.of(), 1.0, VecBuilder.fill(1, 1, 1)));
    camera.sendObservers();
    assertEquals(2, receivedPose.size());
    assertEquals(1, receivedPose.get(1).size());
    assertThrows(
        UnsupportedOperationException.class,
        () ->
            receivedPose
                .get(1)
                .add(
                    new VisionPoseObservation(
                        new Pose2d(), java.util.Set.of(), 2.0, VecBuilder.fill(1, 1, 1))));
  }

  @Test
  public void testSharedFilterPipeline() {
    TestCamera camera = new TestCamera("FilterCam", List.of(), List.of(), List.of());

    // Stale frame
    assertTrue(camera.isStaleFrame(0.0, 1.0));
    assertTrue(camera.isStaleFrame(-0.1, 1.0));
    assertTrue(camera.isStaleFrame(1.0, 1.0));
    assertTrue(camera.isStaleFrame(0.9, 1.0));
    assertFalse(camera.isStaleFrame(1.1, 1.0));

    // Tag count
    assertTrue(camera.isValidTagCount(1));
    assertTrue(camera.isValidTagCount(3));
    assertFalse(camera.isValidTagCount(0));
    assertFalse(camera.isValidTagCount(-1));

    // Bounds without field supplier
    assertNull(camera.getFieldLayoutSupplier());
    assertTrue(
        camera.isWithinFieldBounds(
            new Pose2d(5.0, 5.0, new org.wpilib.math.geometry.Rotation2d()), 0.5));
    assertFalse(camera.isWithinFieldBounds(null, 0.5));

    camera.setFieldLayoutSupplier(() -> null);
    assertTrue(
        camera.isWithinFieldBounds(
            new Pose2d(5.0, 5.0, new org.wpilib.math.geometry.Rotation2d()), 0.5));

    // Bounds with field supplier
    Field field = new Field(null, null, null, null, 16.0, 8.0, null, List.of());
    camera.setFieldLayoutSupplier(() -> field);
    assertEquals(field, camera.getFieldLayoutSupplier().get());

    assertTrue(
        camera.isWithinFieldBounds(
            new Pose2d(5.0, 5.0, new org.wpilib.math.geometry.Rotation2d()), 0.5));
    assertTrue(
        camera.isWithinFieldBounds(
            new Pose2d(-0.4, 0.0, new org.wpilib.math.geometry.Rotation2d()), 0.5));
    assertTrue(
        camera.isWithinFieldBounds(
            new Pose2d(16.4, 8.4, new org.wpilib.math.geometry.Rotation2d()), 0.5));
    assertFalse(
        camera.isWithinFieldBounds(
            new Pose2d(-0.6, 5.0, new org.wpilib.math.geometry.Rotation2d()), 0.5));
    assertFalse(
        camera.isWithinFieldBounds(
            new Pose2d(16.6, 5.0, new org.wpilib.math.geometry.Rotation2d()), 0.5));
    assertFalse(
        camera.isWithinFieldBounds(
            new Pose2d(5.0, -0.6, new org.wpilib.math.geometry.Rotation2d()), 0.5));
    assertFalse(
        camera.isWithinFieldBounds(
            new Pose2d(5.0, 8.6, new org.wpilib.math.geometry.Rotation2d()), 0.5));

    // 3D pose sanity check
    assertFalse(camera.isValid3DPose(null));
    assertTrue(camera.isValid3DPose(new Pose3d(0, 0, 0, new Rotation3d())));
    assertTrue(camera.isValid3DPose(new Pose3d(0, 0, -0.5, new Rotation3d(0.1, 0.1, 0.5))));
    assertFalse(camera.isValid3DPose(new Pose3d(0, 0, -1.0, new Rotation3d()))); // Z too low
    assertFalse(camera.isValid3DPose(new Pose3d(0, 0, 1.5, new Rotation3d()))); // Z too high
    assertFalse(
        camera.isValid3DPose(
            new Pose3d(0, 0, 0, new Rotation3d(Math.toRadians(40), 0, 0)))); // roll too high
    assertFalse(
        camera.isValid3DPose(
            new Pose3d(0, 0, 0, new Rotation3d(0, Math.toRadians(40), 0)))); // pitch too high

    // Ambiguity
    assertTrue(camera.isValidAmbiguity(0.2));
    assertFalse(camera.isValidAmbiguity(0.45));
    assertFalse(camera.isValidAmbiguity(-0.1));

    // Yaw rate
    assertTrue(camera.isValidYawRate(Math.toRadians(180)));
    assertTrue(camera.isValidYawRate(Math.toRadians(720)));
    assertFalse(camera.isValidYawRate(Math.toRadians(730)));
    assertFalse(camera.isValidYawRate(-Math.toRadians(730)));

    // Distance
    assertTrue(camera.isValidDistance(5.0, 10.0));
    assertTrue(camera.isValidDistance(10.0, 10.0));
    assertFalse(camera.isValidDistance(0.0, 10.0));
    assertFalse(camera.isValidDistance(-1.0, 10.0));
    assertFalse(camera.isValidDistance(10.1, 10.0));
  }
}
