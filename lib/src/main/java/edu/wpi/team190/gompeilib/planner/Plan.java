package edu.wpi.team190.gompeilib.planner;

import java.util.ArrayList;
import java.util.HashMap;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;

public class Plan {

  private static final double MIN_DISTANCE = 1e-9;

  private final ArrayList<State> states = new ArrayList<State>();
  private final HashMap<Double, Command> commands = new HashMap<Double, Command>();

  public Plan(Path... paths) {
    // one trapezoid over the whole plan, so it only speeds up at the start and slows down at the
    // end, uses the slowest limits out of all the paths
    double totalDistance = 0;
    double maxVel = Double.POSITIVE_INFINITY;
    double maxAccel = Double.POSITIVE_INFINITY;
    for (Path path : paths) {
      totalDistance += length(path);
      maxVel = Math.min(maxVel, path.getMaxVel());
      maxAccel = Math.min(maxAccel, path.getMaxAccel());
    }
    if (paths.length == 0) {
      return;
    }

    TrapezoidProfile profile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVel, maxAccel));
    TrapezoidProfile.State start = new TrapezoidProfile.State(0, 0);
    TrapezoidProfile.State end = new TrapezoidProfile.State(totalDistance, 0);
    profile.calculate(0, start, end);
    double totalTime = profile.duration();

    int numSteps = Math.max(1, (int) Math.ceil(totalTime * Path.RATE));
    for (int step = 0; step <= numSteps; step++) {
      TrapezoidProfile.State now =
          profile.calculate(Math.min(step / Path.RATE, totalTime), start, end);
      states.add(stateAt(paths, now.position, now.velocity));
    }
  }

  // finds which path the distance lands on and makes the state there
  private static State stateAt(Path[] paths, double distance, double speed) {
    double pathStart = 0;
    for (int i = 0; i < paths.length; i++) {
      Path path = paths[i];
      double length = length(path);
      if (distance <= pathStart + length || i == paths.length - 1) {
        Pose2d startPose = path.getStartPose();
        Pose2d endPose = path.getEndPose();
        double fraction =
            length > MIN_DISTANCE ? Math.min(Math.max((distance - pathStart) / length, 0), 1) : 1;
        double speedPerMeter = length > MIN_DISTANCE ? speed / length : 0;

        Pose2d pose =
            new Pose2d(
                startPose.getTranslation().interpolate(endPose.getTranslation(), fraction),
                startPose.getRotation().interpolate(endPose.getRotation(), fraction));
        ChassisVelocities velocity =
            new ChassisVelocities(
                (endPose.getX() - startPose.getX()) * speedPerMeter,
                (endPose.getY() - startPose.getY()) * speedPerMeter,
                endPose.getRotation().minus(startPose.getRotation()).getRadians() * speedPerMeter);
        return new State(pose, velocity, distance);
      }
      pathStart += length;
    }
    return null;
  }

  private static double length(Path path) {
    return path.getStartPose().getTranslation().getDistance(path.getEndPose().getTranslation());
  }

  public Plan addCommand(double time, Command command) {
    commands.put(time, command);
    return this;
  }

  public HashMap<Double, Command> getCommandMap() {
    return commands;
  }

  public ArrayList<State> getPlan() {
    return states;
  }
  protected ArrayList<Pose2d> getAsPose2dList(){
    ArrayList<Pose2d> poseList = new ArrayList<>();
    states.forEach((State state)->{poseList.add(state.pose());});
    return poseList;
  }
}
// puts together the paths and generates states along the path
