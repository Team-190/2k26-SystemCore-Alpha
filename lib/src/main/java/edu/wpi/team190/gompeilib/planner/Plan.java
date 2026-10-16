package edu.wpi.team190.gompeilib.planner;

import java.util.ArrayList;
import org.wpilib.math.kinematics.ChassisVelocities;

public class Plan {

  private final ArrayList<State> states = new ArrayList<State>();

  public Plan(Path... paths) {
    for (Path path : paths) {
      states.addAll(path.getStates());
    }
    if (!states.isEmpty()) {
      State last = states.remove(states.size() - 1);
      states.add(new State(last.pose(), new ChassisVelocities()));
    }
  }

  public ArrayList<State> getPlan() {
    return states;
  }
}
// puts together the paths and generates states along the path
