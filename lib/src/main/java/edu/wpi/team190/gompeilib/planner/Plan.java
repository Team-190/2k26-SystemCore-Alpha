package edu.wpi.team190.gompeilib.planner;

import java.util.ArrayList;
import java.util.HashMap;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;

public class Plan {

  private final ArrayList<State> states = new ArrayList<State>();
  private final HashMap<Double, Command> commands = new HashMap<Double, Command>();

  public Plan(Path... paths) {
    ArrayList<ArrayList<State>> combinedStatePaths = new ArrayList<>();
    double distanceTotal = 0;
    double vzero = 0;
    for (int i = 0; i < paths.length; i++) {
      ArrayList<State> pathStates = new ArrayList<>();
      Path path = paths[i];
      Pose2d startPose = path.getStartPose();
      Pose2d endPose = path.getStartPose();
      double maxAccel = path.getMaxAccel();
      double maxVel = path.getMaxVel();
      double distance = startPose.getTranslation().getDistance(endPose.getTranslation());
      double time = distance / maxVel;
      if (vzero < maxVel) {
        double timePos =
            ((vzero * -1) + Math.sqrt(vzero * vzero - 4 * maxAccel * distance)) / maxAccel;
        double timeNeg =
            ((vzero * -1) - Math.sqrt(vzero * vzero - 4 * maxAccel * distance)) / maxAccel;
        time = timePos > timeNeg ? timeNeg : timePos;
        vzero = vzero + maxAccel * time;
        double cappedTime = (maxVel - vzero) / maxAccel;
        if (cappedTime > time) {
          double acelDistance = vzero * cappedTime + 0.5 * maxAccel * time * time;
          time = (distance - acelDistance) / maxVel;
        }
      }
      // makes a state every 1 / RATE seconds from start to end (including the end) basically just
      // creates small states along the entire thing
      // at least 1 step, otherwise a time of 0 divides by zero and fills the path with NaN
      int numSteps = Math.max(1, (int) Math.ceil(time * Path.RATE));
      // time gets rounded up to fit the steps baisically (0.73s -> 0.8s) so the velocity matches
      // the
      // states
      double actualTime = numSteps / Path.RATE;

      double deltaX = endPose.getX() - startPose.getX();
      double deltaY = endPose.getY() - startPose.getY();
      double deltaAngle = endPose.getRotation().minus(startPose.getRotation()).getRadians();

      double velocityX = deltaX / actualTime;
      double velocityY = deltaY / actualTime;
      double turnSpeed = deltaAngle / actualTime;
      ChassisVelocities velocity = new ChassisVelocities(velocityX, velocityY, turnSpeed);

      for (int step = 0; step <= numSteps; step++) {
        double fraction = (double) step / numSteps; // 0 at start, 1 at end
        Pose2d pose =
            new Pose2d(
                startPose.getTranslation().interpolate(endPose.getTranslation(), fraction),
                startPose.getRotation().interpolate(endPose.getRotation(), fraction));
        pathStates.add(new State(pose, velocity, distanceTotal + (fraction * distance)));
      }
      combinedStatePaths.add(pathStates);
    }
    for (ArrayList<State> pathStates : combinedStatePaths) {
      // make sure no stopping when connecting, but it just like goes through

      if (!states.isEmpty() && !pathStates.isEmpty()) {
        State previousEnd = states.get(states.size() - 1);
        State currentStart = pathStates.get(0);
        if (previousEnd.pose().equals(currentStart.pose())) {
          states.remove(states.size() - 1);
        }
      }
      states.addAll(pathStates);
    }
    TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(4, 1));
    TrapezoidProfile.State start = new TrapezoidProfile.State(0, 0);
    TrapezoidProfile.State end = new TrapezoidProfile.State(distanceTotal, 0);
    states.forEach(
        (State state) -> {
          state.velocity().times(profile.calculate(state.distance(), start, end).position);
        });
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
}
// puts together the paths and generates states along the path
