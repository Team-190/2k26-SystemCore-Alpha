package edu.wpi.team190.gompeilib.planner;

import java.util.ArrayList;

public class Plan {

  private final ArrayList<State> states = new ArrayList<State>();

  public Plan(Path... paths) {
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

      states.addAll(pathStates);
    }
  }

  public ArrayList<State> getPlan() {
    return states;
  }
}
// puts together the paths and generates states along the path
