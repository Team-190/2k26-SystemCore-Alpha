package edu.wpi.team190.gompeilib.core.planner.paths;

import java.util.ArrayList;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.util.Pair;

public abstract class Path {

  public Pose2d startPose;
  public Pose2d endPose;
  public PathConstraints constraints;
  public ArrayList<Pair<Double, Command>> eventList = new ArrayList<Pair<Double, Command>>();

  abstract Pose2d sample(double time);

  abstract Pose2d sample(Pose2d pose);

  public Path withCommand(double time, Command command) {
    eventList.add(Pair.of(time, command));
    return this;
  }
}
