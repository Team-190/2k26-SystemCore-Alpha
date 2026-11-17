package edu.wpi.team190.gompeilib.core.planner.paths;

import java.util.ArrayList;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.util.Pair;

import edu.wpi.team190.gompeilib.core.planner.States;

public abstract class Path {

  public final Pose2d startPose;
  public final Pose2d endPose;
  public final PathConstraints constraints;
  public ArrayList<Pair<Double, Command>> eventList = new ArrayList<Pair<Double, Command>>();

  public Path(Pose2d startPose,Pose2d endPose, PathConstraints constraints){
    this.startPose = startPose;
    this.endPose = endPose;
    this.constraints = constraints;
  }
  abstract States sample(double time);

  abstract States sample(Pose2d pose);

  public Path withCommand(double time, Command command) {
    eventList.add(Pair.of(time, command));
    return this;
  }
}
