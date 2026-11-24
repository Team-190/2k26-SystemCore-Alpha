package edu.wpi.team190.gompeilib.core.planner.paths;

import java.util.ArrayList;
import java.util.List;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.util.Pair;

public abstract class Path {

  public static final int SAMPLE_RATE = 100;

  public final Pose2d startPose;
  public final Pose2d endPose;
  public final PathConstraints constraints;
  public ArrayList<Pair<Double, Command>> eventList = new ArrayList<Pair<Double, Command>>();

  public Path(Pose2d startPose, Pose2d endPose, PathConstraints constraints) {
    this.startPose = startPose;
    this.endPose = endPose;
    this.constraints = constraints;
  }

  abstract Pose2d sample(double time);

  abstract Pose2d sample(Pose2d pose, ChassisVelocities vel);

  public Path withCommand(double time, Command command) {
    eventList.add(Pair.of(time, command));
    return this;
  }

  public double length() {
    return length(SAMPLE_RATE);
  }

  public double length(int subdivisions) {
    int steps = Math.max(1, subdivisions);
    Pose2d previous = sample(0.0);
    double total = 0.0;
    for (int i = 1; i <= steps; i++) {
      Pose2d current = sample(i / (double) steps);
      total += distance(previous, current);
      previous = current;
    }
    return total;
  }

  public double straightLineDistance() {
    return distance(startPose, endPose);
  }

  public List<Pose2d> sampleUniform(int count) {
    int points = Math.max(2, count);
    List<Pose2d> samples = new ArrayList<>(points);
    for (int i = 0; i < points; i++) {
      samples.add(sample(i / (double) (points - 1)));
    }
    return samples;
  }

  private static double distance(Pose2d a, Pose2d b) {
    return Math.hypot(a.getX() - b.getX(), a.getY() - b.getY());
  }

  Pose2d sample(Pose2d pose) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'sample'");
  }
}
