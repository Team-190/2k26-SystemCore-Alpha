package edu.wpi.team190.gompeilib.robot.subsystems.vision.camera;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.team190.gompeilib.robot.subsystems.vision.VisionConstants;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.data.VisionMultiTxTyObservation;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.data.VisionPoseObservation;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.CameraIO;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.CameraIOGompeiVision;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.numbers.N5;
import org.wpilib.system.Timer;

public class CameraGompeiVisionTest {

  @Test
  public void testGompeiVisionCamera() {
    CameraIOGompeiVision io = mock(CameraIOGompeiVision.class);

    Matrix<N3, N3> cameraMatrix = new Matrix<>(new SimpleMatrix(3, 3));
    Matrix<N5, N1> distortion = new Matrix<>(new SimpleMatrix(5, 1));
    Pose3d cameraPoseRel = new Pose3d(0.5, 0.0, 0.5, new org.wpilib.math.geometry.Rotation3d());

    VisionConstants.GompeiVisionConfig config =
        VisionConstants.GompeiVisionConfig.builder()
            .key("gompei")
            .cameraType(CameraType.THRIFTYCAM)
            .cameraMatrix(cameraMatrix)
            .distortionCoefficients(distortion)
            .singletagXYStdev(0.1)
            .thetaStdev(0.2)
            .multitagXYStdev(0.01)
            .robotRelativePose(cameraPoseRel)
            .build();

    FieldTag tag1 =
        new FieldTag(1, new Pose3d(1.0, 2.0, 3.0, new org.wpilib.math.geometry.Rotation3d()));
    FieldTag tag2 =
        new FieldTag(2, new Pose3d(5.0, 5.0, 3.0, new org.wpilib.math.geometry.Rotation3d()));
    Field layout = new Field(null, null, null, null, 16.0, 8.0, null, List.of(tag1, tag2));

    List<VisionPoseObservation> poses = new ArrayList<>();
    List<VisionMultiTxTyObservation> txtys = new ArrayList<>();

    Consumer<List<VisionPoseObservation>> poseObserver = poses::addAll;
    Consumer<List<VisionMultiTxTyObservation>> txtyObserver = txtys::addAll;

    CameraGompeiVision camera =
        new CameraGompeiVision(
            io,
            config,
            () -> layout,
            0.5,
            () -> new Pose2d(0.0, 0.0, new Rotation2d()),
            List.of(poseObserver),
            List.of(txtyObserver));

    assertEquals("gompei", camera.getName());
    assertEquals(cameraPoseRel, camera.getRealTimeCameraPose());
    assertEquals(cameraPoseRel, camera.getCurrentCameraPose());

    try (MockedStatic<Logger> _ = mockStatic(Logger.class);
        MockedStatic<DriverStationBackend> mockDS = mockStatic(DriverStationBackend.class)) {

      // --- Test empty frames ---
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames = new double[][] {new double[] {}};
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      camera.periodic();
      assertTrue(poses.isEmpty());

      // --- Test Invalid Switch Case ---
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames = new double[][] {new double[] {3.0}};
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      camera.periodic();
      mockDS.verify(
          () -> DriverStationBackend.reportWarning("FAILED TO CAPTURE FRAMES", false), times(1));

      // --- Test Case 1: One Pose (Valid, Inside Field) ---
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        1.0,
                        0.0, // values[0]=1, values[1]=0
                        1.0,
                        2.0,
                        0.0, // translation (1, 2, 0)
                        1.0,
                        0.0,
                        0.0,
                        0.0, // quaternion (w=1, x=0, y=0, z=0)
                        1.0, // tagId = 1
                        0.1,
                        0.2,
                        0.3,
                        0.4,
                        0.5,
                        0.6,
                        0.7,
                        0.8, // corners
                        2.0 // distance = 2.0
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      txtys.clear();
      camera.periodic();

      assertFalse(poses.isEmpty());
      assertEquals(1, poses.size());
      assertEquals(1, txtys.size());
      assertEquals(1, poses.get(0).tagIds().size());
      assertTrue(poses.get(0).tagIds().contains(1));

      // --- Test Case 1: Outside Field boundary ---
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        1.0, 0.0, 20.0, 2.0,
                        0.0, // X is 20, field is 16. With 0.5 margin, it's outside.
                        1.0, 0.0, 0.0, 0.0, 1.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 2.0
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      camera.periodic();
      assertTrue(poses.isEmpty());

      // --- Test Case 2: Multiple Poses ---
      // We will provide two poses, error0 = 0.05, error1 = 0.5 (ambiguity resolution should favor
      // pose 0)
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        2.0, 0.05, // values[0]=2, error0=0.05
                        1.0, 2.0, 0.0, // pose0 translation
                        1.0, 0.0, 0.0, 0.0, // pose0 quaternion (no rot)
                        0.5, // error1=0.5
                        1.0, 5.0, 0.0, // pose1 translation
                        1.0, 0.0, 0.0, 0.0, // pose1 quaternion
                        1.0, // tagId=1
                        0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 2.0
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      camera.periodic();
      assertFalse(poses.isEmpty());
      // Pose 0 is selected, robot X should be around 1.0 - camera translation
      assertTrue(poses.get(0).pose().getX() < 2.0);

      // Test Case 2: Ambiguity favoring pose 1
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        2.0, 0.5, // values[0]=2, error0=0.5
                        1.0, 2.0, 0.0, // pose0 translation
                        1.0, 0.0, 0.0, 0.0, // pose0 quaternion
                        0.05, // error1=0.05
                        1.5, 2.0, 0.0, // pose1 translation
                        1.0, 0.0, 0.0, 0.0, // pose1 quaternion
                        1.0, // tagId=1
                        0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 2.0
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      camera.periodic();
      assertFalse(poses.isEmpty());

      // --- Test frame present but reporting zero targets: covers the values[0]==0 skip ---
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames = new double[][] {new double[] {0.0, 1.0, 2.0}};
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      camera.periodic();
      assertTrue(poses.isEmpty());

      // --- Test Case 2: Ambiguous errors (neither pose confident enough to select) ---
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        2.0,
                        0.3, // values[0]=2, error0=0.3
                        1.0,
                        2.0,
                        0.0, // pose0 translation
                        1.0,
                        0.0,
                        0.0,
                        0.0, // pose0 quaternion
                        0.3, // error1=0.3 (too close to error0 to be confident)
                        1.0,
                        5.0,
                        0.0, // pose1 translation
                        1.0,
                        0.0,
                        0.0,
                        0.0 // pose1 quaternion
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      camera.periodic();
      assertTrue(poses.isEmpty());

      // --- Test Case 2: Confident selection, resolved by rotation proximity favoring pose 0 ---
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        2.0, 0.05, // values[0]=2, error0=0.05
                        1.0, 2.0, 0.0, // pose0 translation
                        1.0, 0.0, 0.0, 0.0, // pose0 quaternion (no rotation, matches current pose)
                        0.5, // error1=0.5
                        1.0, 5.0, 0.0, // pose1 translation
                        0.7071, 0.0, 0.0, 0.7071, // pose1 quaternion (90 degrees about Z)
                        1.0, // tagId=1
                        0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 2.0
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      camera.periodic();
      assertFalse(poses.isEmpty());

      // --- Test Case 1: Far outside field boundary (unambiguous) ---
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        1.0, 0.0, 1.0e7, 2.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.1, 0.2, 0.3, 0.4, 0.5,
                        0.6, 0.7, 0.8, 2.0
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      camera.periodic();
      assertTrue(poses.isEmpty());

      // --- Test Case 1: Outside field via negative X ---
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        1.0, 0.0, -1.0e7, 2.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.1, 0.2, 0.3, 0.4,
                        0.5, 0.6, 0.7, 0.8, 2.0
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      camera.periodic();
      assertTrue(poses.isEmpty());

      // --- Test Case 1: Outside field via negative Y ---
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        1.0, 0.0, 1.0, -1.0e7, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.1, 0.2, 0.3, 0.4,
                        0.5, 0.6, 0.7, 0.8, 2.0
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      camera.periodic();
      assertTrue(poses.isEmpty());

      // --- Test Case 1: Outside field via large positive Y ---
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        1.0, 0.0, 1.0, 1.0e7, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.1, 0.2, 0.3, 0.4, 0.5,
                        0.6, 0.7, 0.8, 2.0
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      camera.periodic();
      assertTrue(poses.isEmpty());

      // --- Test Case 1: Valid, in-bounds pose referencing a tag not in the field layout ---
      // (covers the tagPoses.isEmpty() branch, since tag 99 isn't in the layout)
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        1.0, 0.0, 1.0, 2.0, 0.0, 1.0, 0.0, 0.0, 0.0, 99.0, 0.1, 0.2, 0.3, 0.4, 0.5,
                        0.6, 0.7, 0.8, 2.0
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      camera.periodic();
      assertEquals(0, poses.size()); // Tier 1 #6 fix: skip when totalTargets == 0

      // --- Test Case 1: Two valid tags, both present in the field layout (multi-tag branch) ---
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        1.0,
                        0.0,
                        1.0,
                        2.0,
                        0.0,
                        1.0,
                        0.0,
                        0.0,
                        0.0, // header
                        1.0,
                        0.1,
                        0.2,
                        0.3,
                        0.4,
                        0.5,
                        0.6,
                        0.7,
                        0.8,
                        2.0, // tag 1 chunk
                        2.0,
                        0.1,
                        0.2,
                        0.3,
                        0.4,
                        0.5,
                        0.6,
                        0.7,
                        0.8,
                        3.0 // tag 2 chunk
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      txtys.clear();
      camera.periodic();
      assertFalse(poses.isEmpty());
      assertEquals(2, txtys.size());
      assertEquals(2, poses.get(0).tagIds().size());

      // --- Test Case 1: More tag data chunks than valid targets, and a misaligned array length
      // --- (covers the "more tag data than expected" and "not a multiple of 10" warnings) ---
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        1.0,
                        0.0,
                        1.0,
                        2.0,
                        0.0,
                        1.0,
                        0.0,
                        0.0,
                        0.0, // header
                        1.0,
                        0.1,
                        0.2,
                        0.3,
                        0.4,
                        0.5,
                        0.6,
                        0.7,
                        0.8,
                        2.0, // tag 1 chunk (valid)
                        99.0,
                        0.1,
                        0.2,
                        0.3,
                        0.4,
                        0.5,
                        0.6,
                        0.7,
                        0.8,
                        3.0, // tag 99 (not in layout)
                        0.0 // dangling trailing value, misaligns the chunk length
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      txtys.clear();
      camera.periodic();
      assertFalse(poses.isEmpty());
      assertEquals(1, txtys.size());
    }
  }

  @Test
  public void testCornerParsingWithMultipleFrames() {
    CameraIOGompeiVision io = mock(CameraIOGompeiVision.class);
    Matrix<N3, N3> cameraMatrix = new Matrix<>(new SimpleMatrix(3, 3));
    Matrix<N5, N1> distortion = new Matrix<>(new SimpleMatrix(5, 1));
    Pose3d cameraPoseRel = new Pose3d(0.5, 0.0, 0.0, new org.wpilib.math.geometry.Rotation3d());

    VisionConstants.GompeiVisionConfig config =
        VisionConstants.GompeiVisionConfig.builder()
            .key("gompei-multi")
            .cameraType(CameraType.THRIFTYCAM)
            .cameraMatrix(cameraMatrix)
            .distortionCoefficients(distortion)
            .singletagXYStdev(0.1)
            .thetaStdev(0.2)
            .multitagXYStdev(0.01)
            .robotRelativePose(cameraPoseRel)
            .build();

    FieldTag tag1 =
        new FieldTag(1, new Pose3d(1.0, 2.0, 0.0, new org.wpilib.math.geometry.Rotation3d()));
    Field layout = new Field(null, null, null, null, 16.0, 8.0, null, List.of(tag1));

    List<VisionPoseObservation> poses = new ArrayList<>();
    List<VisionMultiTxTyObservation> txtys = new ArrayList<>();

    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      CameraGompeiVision camera =
          new CameraGompeiVision(
              io,
              config,
              () -> layout,
              0.5,
              () -> Pose2d.ZERO,
              List.of(poses::addAll),
              List.of(txtys::addAll));

      // Feed 5 frames. Before Bug #2 fix, when i >= 4, indexing tx[i] threw
      // ArrayIndexOutOfBoundsException!
      double[] frameTemplate =
          new double[] {
            1.0,
            0.0,
            1.0,
            2.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0, // header
            1.0, // tagId
            10.0,
            11.0,
            20.0,
            21.0,
            30.0,
            31.0,
            40.0,
            41.0, // 4 corner pairs: (10,11), (20,21), (30,31), (40,41)
            2.0 // distance
          };

      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0, 2.0, 3.0, 4.0, 5.0};
                inputs.frames =
                    new double[][] {
                      frameTemplate, frameTemplate, frameTemplate, frameTemplate, frameTemplate
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      assertDoesNotThrow(() -> camera.periodic());
      assertEquals(5, poses.size());
      assertEquals(5, txtys.size());

      // Verify corner parsing indexes j correctly, not outer loop index i
      VisionMultiTxTyObservation obs = txtys.get(4); // 5th frame (i=4)
      assertArrayEquals(new double[] {10.0, 20.0, 30.0, 40.0}, obs.tx(), 1e-6);
      assertArrayEquals(new double[] {11.0, 21.0, 31.0, 41.0}, obs.ty(), 1e-6);
    }
  }

  @Test
  public void testAmbiguityResolutionLogic() {
    CameraIOGompeiVision io = mock(CameraIOGompeiVision.class);
    Matrix<N3, N3> cameraMatrix = new Matrix<>(new SimpleMatrix(3, 3));
    Matrix<N5, N1> distortion = new Matrix<>(new SimpleMatrix(5, 1));
    Pose3d cameraPoseRel = new Pose3d(0.0, 0.0, 0.0, new org.wpilib.math.geometry.Rotation3d());

    VisionConstants.GompeiVisionConfig config =
        VisionConstants.GompeiVisionConfig.builder()
            .key("gompei-ambiguity")
            .cameraType(CameraType.THRIFTYCAM)
            .cameraMatrix(cameraMatrix)
            .distortionCoefficients(distortion)
            .singletagXYStdev(0.1)
            .thetaStdev(0.2)
            .multitagXYStdev(0.01)
            .robotRelativePose(cameraPoseRel)
            .build();

    FieldTag tag1 =
        new FieldTag(1, new Pose3d(5.0, 5.0, 0.0, new org.wpilib.math.geometry.Rotation3d()));
    Field layout = new Field(null, null, null, null, 16.0, 8.0, null, List.of(tag1));

    List<VisionPoseObservation> poses = new ArrayList<>();
    Pose2d[] robotHeadingSupplier = {
      new Pose2d(0, 0, Rotation2d.fromDegrees(180))
    }; // Heading near 180 deg (closer to pose 1)
    double[] currentTimestamp = {1.0};

    try (MockedStatic<Logger> _ = mockStatic(Logger.class);
        MockedStatic<Timer> mockTimer = mockStatic(Timer.class)) {
      mockTimer.when(Timer::getTimestamp).thenAnswer(_ -> currentTimestamp[0]);
      CameraGompeiVision camera =
          new CameraGompeiVision(
              io,
              config,
              () -> layout,
              0.5,
              () -> robotHeadingSupplier[0],
              List.of(poses::addAll),
              List.of());

      // Case 1: error0=0.01, error1=1.0 (error0 < 0.4 * error1 -> unambiguous pose 0)
      // Even though robotHeading is 180 deg (closer to pose 1 rotation of 180 deg than pose 0
      // rotation of 0 deg),
      // Bug #4 fix ensures pose 0 is selected because error0 is unambiguously better.
      currentTimestamp[0] = 1.0;
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        2.0, 0.01, // values[0]=2, error0=0.01
                        2.0, 3.0, 0.0, // pose0 at (2, 3), rot 0
                        1.0, 0.0, 0.0, 0.0, 1.0, // error1=1.0
                        8.0, 8.0, 0.0, // pose1 at (8, 8), rot 180
                        0.0, 0.0, 0.0, 1.0, // rot 180 deg
                        1.0, 0, 0, 0, 0, 0, 0, 0, 0, 2.0
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      camera.periodic();
      assertEquals(1, poses.size());
      assertEquals(2.0, poses.get(0).pose().getX(), 0.01);
      assertEquals(3.0, poses.get(0).pose().getY(), 0.01);

      // Case 2: error1=0.01, error0=1.0 (error1 < 0.4 * error0 -> unambiguous pose 1)
      currentTimestamp[0] = 2.0;
      robotHeadingSupplier[0] =
          new Pose2d(0, 0, Rotation2d.fromDegrees(0)); // heading closer to pose 0
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {2.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        2.0, 1.0, // error0=1.0
                        2.0, 3.0, 0.0, // pose0
                        1.0, 0.0, 0.0, 0.0, 0.01, // error1=0.01
                        8.0, 7.0, 0.0, // pose1
                        0.0, 0.0, 0.0, 1.0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 2.0
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      camera.periodic();
      assertEquals(1, poses.size());
      assertEquals(8.0, poses.get(0).pose().getX(), 0.01);
      assertEquals(7.0, poses.get(0).pose().getY(), 0.01);

      // Case 3: Ambiguous frame (error0=0.5, error1=0.5) -> disambiguate using robot gyro heading
      currentTimestamp[0] = 3.0;
      robotHeadingSupplier[0] =
          new Pose2d(0, 0, Rotation2d.fromDegrees(10)); // closer to pose 0 (0 deg)
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {3.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        2.0, 0.5, // error0=0.5
                        2.0, 3.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, // error1=0.5
                        8.0, 7.0, 0.0, 0.0, 0.0, 0.0, 1.0, // 180 deg
                        1.0, 0, 0, 0, 0, 0, 0, 0, 0, 2.0
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      camera.periodic();
      assertEquals(1, poses.size());
      assertEquals(2.0, poses.get(0).pose().getX(), 0.01); // Pose 0 chosen

      currentTimestamp[0] = 4.0;
      robotHeadingSupplier[0] =
          new Pose2d(0, 0, Rotation2d.fromDegrees(170)); // closer to pose 1 (180 deg)
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {4.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        2.0, 0.5, // error0=0.5
                        2.0, 3.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, // error1=0.5
                        8.0, 7.0, 0.0, 0.0, 0.0, 0.0, 1.0, // 180 deg
                        1.0, 0, 0, 0, 0, 0, 0, 0, 0, 2.0
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));
      poses.clear();
      camera.periodic();
      assertEquals(1, poses.size());
      assertEquals(8.0, poses.get(0).pose().getX(), 0.01); // Pose 1 chosen
    }
  }

  @Test
  public void testFlyingRobotPoseRejection() {
    CameraIOGompeiVision io = mock(CameraIOGompeiVision.class);
    Matrix<N3, N3> cameraMatrix = new Matrix<>(new SimpleMatrix(3, 3));
    Matrix<N5, N1> distortion = new Matrix<>(new SimpleMatrix(5, 1));
    Pose3d cameraPoseRel = new Pose3d(0.0, 0.0, 0.0, new org.wpilib.math.geometry.Rotation3d());

    VisionConstants.GompeiVisionConfig config =
        VisionConstants.GompeiVisionConfig.builder()
            .key("gompei-flying")
            .cameraType(CameraType.THRIFTYCAM)
            .cameraMatrix(cameraMatrix)
            .distortionCoefficients(distortion)
            .singletagXYStdev(0.1)
            .thetaStdev(0.2)
            .multitagXYStdev(0.01)
            .robotRelativePose(cameraPoseRel)
            .build();

    FieldTag tag1 =
        new FieldTag(1, new Pose3d(5.0, 5.0, 0.0, new org.wpilib.math.geometry.Rotation3d()));
    Field layout = new Field(null, null, null, null, 16.0, 8.0, null, List.of(tag1));

    List<VisionPoseObservation> poses = new ArrayList<>();

    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      CameraGompeiVision camera =
          new CameraGompeiVision(
              io, config, () -> layout, 0.5, () -> Pose2d.ZERO, List.of(poses::addAll), List.of());

      // Flying robot: Z = 4.0 meters
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .GompeiVisionIOInputsAutoLogged)
                        invocation.getArgument(0);
                inputs.timestamps = new double[] {1.0};
                inputs.frames =
                    new double[][] {
                      new double[] {
                        1.0, 0.0, 2.0, 3.0, 4.0, // Z = 4.0 (flying)
                        1.0, 0.0, 0.0, 0.0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 2.0
                      }
                    };
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.GompeiVisionIOInputs.class));

      poses.clear();
      camera.periodic();
      assertTrue(poses.isEmpty());
    }
  }
}
