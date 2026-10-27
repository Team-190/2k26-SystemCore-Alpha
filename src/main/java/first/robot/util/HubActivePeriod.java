package first.robot.util;

import java.util.Optional;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.system.Timer;

public class HubActivePeriod {

  @RequiredArgsConstructor
  public enum Shift {
    AUTO(0.0, new ActiveSchedule(true, true)),
    TRANSITION(0.0, new ActiveSchedule(true, true)),
    SHIFT1(10.0, new ActiveSchedule(false, true)),
    SHIFT2(35.0, new ActiveSchedule(false, false)),
    SHIFT3(60.0, new ActiveSchedule(false, true)),
    SHIFT4(85.0, new ActiveSchedule(false, false)),
    ENDGAME(110.0, new ActiveSchedule(true, true)),
    END(140.0, new ActiveSchedule(false, false));

    @Getter private final double startTime;
    private final ActiveSchedule schedule;

    private static class ActiveSchedule {
      private final boolean activeSchedule, inactiveSchedule;

      public ActiveSchedule(boolean alwaysActive, boolean active) {
        if (alwaysActive) {
          activeSchedule = active;
          inactiveSchedule = active;
        } else {
          activeSchedule = active;
          inactiveSchedule = !active;
        }
      }
    }

    public boolean isActive(Alliance alliance, Alliance firstActiveAlliance) {
      return alliance == firstActiveAlliance ? schedule.activeSchedule : schedule.inactiveSchedule;
    }

    public double getDuration() {
      if (ordinal() + 1 >= shifts.length) {
        return 0.0;
      }
      return shifts[ordinal() + 1].getStartTime() - startTime;
    }
  }

  private static final double TELEOP_LENGTH = 140.0;
  private static final Timer shiftTimer = new Timer();
  private static final Shift[] shifts = Shift.values();

  public static Optional<Alliance> getFirstActiveAlliance() {
    // var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    String message = MatchState.getGameData().orElse("");
    if (message.isEmpty()) {
      return Optional.empty();
    }
    return switch (message.charAt(0)) {
      case 'R' -> Optional.of(Alliance.BLUE);
      case 'B' -> Optional.of(Alliance.RED);
      default -> Optional.empty();
    };
  }

  /** Call at start of teleop */
  public static void initialize() {
    shiftTimer.restart();
  }

  private static double getTeleopElapsed() {
    double matchTime = MatchState.getMatchTime();
    return matchTime >= 0 ? TELEOP_LENGTH - matchTime : shiftTimer.get();
  }

  public static Shift getCurrentShift() {
    // Auto always returns AUTO
    if (RobotState.isAutonomous()) {
      return Shift.AUTO;
    }

    double time = getTeleopElapsed();

    Shift current = Shift.AUTO;

    for (Shift shift : shifts) {
      if (time >= shift.getStartTime()) {
        current = shift;
      } else {
        break;
      }
    }

    return current;
  }

  public static boolean isHubActive() {
    Shift shift = getCurrentShift();
    Optional<Alliance> firstActive = getFirstActiveAlliance();
    Optional<Alliance> alliance = MatchState.getAlliance();

    if (shift == Shift.AUTO || firstActive.isEmpty() || alliance.isEmpty()) {
      return true;
    }

    return shift.isActive(alliance.get(), firstActive.get());
  }

  public static double getShiftTimeRemaining() {
    Shift shift = getCurrentShift();
    return shift.ordinal() != shifts.length - 1
        ? shift.getDuration() - (getTeleopElapsed() - shift.getStartTime())
        : 0.0;
  }
}
