package edu.wpi.team190.gompeilib.planner;

import java.util.ArrayList;
import java.util.HashMap;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.trajectory.TrapezoidProfile;

public class Plan {

  private final ArrayList<State> states = new ArrayList<State>();
  private final HashMap<Double, Command> commands = new HashMap<Double, Command>();

  public Plan(Path... paths) {
    double distance = 0;
    HashMap<State,Double> distanceMap = new HashMap<>();
    for (Path path : paths) {
      ArrayList<State> pathStates = path.getStates();
      // make sure no stopping when connecting, but it just like goes through
      
      
      if (!states.isEmpty() && !pathStates.isEmpty()) {
        State previousEnd = states.get(states.size() - 1);
        State currentStart = pathStates.get(0);
        if (previousEnd.pose().equals(currentStart.pose())) {
          states.remove(states.size() - 1);
        }
      }
      double fracDistance = distance + (path.getDistance());
      pathStates.forEach((State state)->{
        distanceMap.put(state, fracDistance + (path.getDistance())*state.distance());
      });
      states.addAll(pathStates);
      distance += path.getDistance();
    }
    TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(4, 1));
    TrapezoidProfile.State start = new TrapezoidProfile.State(0,0);
    TrapezoidProfile.State end = new TrapezoidProfile.State(distance,0);
    states.forEach((State state)->{
      state.velocity().times(profile.calculate(distanceMap.get(state), start, end).position);

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
