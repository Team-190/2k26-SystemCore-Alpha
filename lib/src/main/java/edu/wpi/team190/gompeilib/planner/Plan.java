package edu.wpi.team190.gompeilib.planner;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

public class Plan {

  private final ArrayList<State> states = new ArrayList<State>();
  private final HashMap<Double, Command> commands = new HashMap<Double,Command>();

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

  public Plan addCommand(double time, Command command){
    commands.put(time,command);
    return this;
  }

  public HashMap<Double, Command> getCommandMap(){
    return commands;
  }
  public ArrayList<State> getPlan() {
    return states;
  }
}
// puts together the paths and generates states along the path
