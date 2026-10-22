package edu.wpi.team190.gompeilib.robot.subsystems.vision.data;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

public record VisionSingleTxTyObservation(
    int tagId, Angle tx, Angle ty, Distance distance, Time timestamp, Pose3d cameraPose) {
  public VisionSingleTxTyObservation(
      int tagId,
      double txDegrees,
      double tyDegrees,
      double distanceMeters,
      double timestampSeconds,
      Pose3d cameraPose) {
    this(
        tagId,
        Degrees.of(txDegrees),
        Degrees.of(tyDegrees),
        Meters.of(distanceMeters),
        Seconds.of(timestampSeconds),
        cameraPose);
  }
}
