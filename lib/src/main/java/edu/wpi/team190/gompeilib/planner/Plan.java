package edu.wpi.team190.gompeilib.planner;

import edu.wpi.team190.gompeilib.core.GompeiLib;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.wpilib.command2.Command;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;

public class Plan {

  private static final double DEFAULT_SAMPLE_RATE_HZ = 200.0;
  private static final double MIN_DISTANCE = 1e-9;

  private final ArrayList<State> states = new ArrayList<State>();
  private final HashMap<Double, Command> commands = new HashMap<Double, Command>();
  private final List<Path> paths;
  private final double[] pathEndTimes;
  private final double sampleRateHz;

  /**
   * Captures the control loop rate once, using 200 Hz with a warning if unavailable or invalid.
   * This gives one stored sample per loop; playback still interpolates for timing jitter.
   */
  public Plan(Path... paths) {
    this(defaultSampleRateHz(), paths);
  }

  private static double defaultSampleRateHz() {
    double loopPeriod;
    try {
      loopPeriod = GompeiLib.getLoopPeriod();
    } catch (IllegalStateException exception) {
      DriverStationErrors.reportWarning(
          "GompeiPlanner: GompeiLib not initialized; using 200 Hz for this Plan. "
              + "Call GompeiLib.init() before constructing Plans.",
          false);
      return DEFAULT_SAMPLE_RATE_HZ;
    }
    double rate = 1.0 / loopPeriod;
    if (!Double.isFinite(loopPeriod) || loopPeriod <= 0 || !Double.isFinite(rate)) {
      DriverStationErrors.reportWarning(
          "GompeiPlanner: Invalid GompeiLib loop period; using 200 Hz for this Plan.", false);
      return DEFAULT_SAMPLE_RATE_HZ;
    }
    return rate;
  }

  /**
   * Builds approximately duration * sampleRateHz + 1 states. Memory and construction time grow with
   * that count (200 Hz stores about 20 times as many states as 10 Hz). Callers may choose a lower
   * rate for long plans, trading interpolation accuracy for fewer stored states.
   *
   * @throws IllegalArgumentException if the rate is not finite and positive, or the sample count
   *     cannot fit in a list
   */
  public Plan(double sampleRateHz, Path... paths) {
    if (!Double.isFinite(sampleRateHz) || sampleRateHz <= 0) {
      throw new IllegalArgumentException("Sample rate must be finite and positive (Hz)");
    }
    this.paths = List.of(paths);
    this.pathEndTimes = new double[paths.length];
    this.sampleRateHz = sampleRateHz;
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

    double stepCount = Math.ceil(totalTime * sampleRateHz);
    if (!Double.isFinite(stepCount) || stepCount > Integer.MAX_VALUE - 2) {
      throw new IllegalArgumentException("Plan has too many samples; use a lower sample rate");
    }
    int numSteps = Math.max(1, (int) stepCount);
    for (int step = 0; step <= numSteps; step++) {
      TrapezoidProfile.State now =
          profile.calculate(Math.min(step / sampleRateHz, totalTime), start, end);
      states.add(stateAt(paths, now.position, now.velocity));
    }

    double pathEnd = 0;
    for (int i = 0; i < paths.length; i++) {
      pathEnd += length(paths[i]);
      int step = 0;
      while (step < states.size() - 1 && states.get(step).distance() < pathEnd) {
        step++;
      }
      pathEndTimes[i] = step / sampleRateHz;
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
        return new State(pose, fraction == 1 ? path.getEndVelocity() : velocity, distance);
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

  public Path getPath(int pathIndex) {
    return paths.get(pathIndex);
  }

  public int getPathCount() {
    return paths.size();
  }

  public double getPathEndTime(int pathIndex) {
    return pathEndTimes[pathIndex];
  }

  public int pathIndexAt(double time) {
    for (int i = 0; i < pathEndTimes.length - 1; i++) {
      if (time < pathEndTimes[i]) {
        return i;
      }
    }
    return pathEndTimes.length - 1;
  }

  public ArrayList<State> getPlan() {
    return states;
  }

  public double getSampleRateHz() {
    return sampleRateHz;
  }

  /**
   * Interpolates at the plan's captured sample rate, clamping times outside the plan to its
   * endpoints. NaN is rejected; an empty plan cannot be sampled.
   */
  public State sample(double time) {
    if (Double.isNaN(time)) {
      throw new IllegalArgumentException("Sample time must not be NaN");
    }
    if (states.isEmpty()) {
      throw new IllegalStateException("Cannot sample an empty plan");
    }
    int lastIndex = states.size() - 1;
    double exactIndex = Math.max(0, Math.min(time * sampleRateHz, lastIndex));
    int index = (int) exactIndex;
    State before = states.get(index);
    if (index == lastIndex) {
      return before;
    }
    State after = states.get(index + 1);
    double fraction = exactIndex - index;
    return new State(
        before.pose().interpolate(after.pose(), fraction),
        before.velocity().interpolate(after.velocity(), fraction),
        before.distance() + (after.distance() - before.distance()) * fraction);
  }

  public double speedAt(double time) {
    ChassisVelocities velocity = sample(time).velocity();
    return Math.hypot(velocity.vx, velocity.vy);
  }

  protected ArrayList<Pose2d> getAsPose2dList() {
    ArrayList<Pose2d> poseList = new ArrayList<>();
    states.forEach(
        (State state) -> {
          poseList.add(state.pose());
        });
    return poseList;
  }
}
// puts together the paths and generates states along the path
