package edu.wpi.team190.gompeilib.planner;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import java.util.ArrayList;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;

class PlannerTest {
  @ParameterizedTest
  @ValueSource(doubles = {2, 2.005, 100})
  void appliesFinalSampleAndTriggersFinalCommandWhenLoopReachesOrPassesEnd(double time) {
    assertCompletesAtFinalSample(time, 0);
  }

  @ParameterizedTest
  @ValueSource(doubles = {0, 0.005})
  void endRadiusJumpAppliesFinalSampleAndTriggersFinalCommand(double time) {
    assertCompletesAtFinalSample(time, 0.1);
  }

  private void assertCompletesAtFinalSample(double time, double endRadius) {
    CommandScheduler scheduler = mock(CommandScheduler.class);
    try (MockedStatic<Logger> logger = mockStatic(Logger.class);
        MockedStatic<CommandScheduler> schedulers = mockStatic(CommandScheduler.class);
        MockedConstruction<Timer> timers =
            mockConstruction(Timer.class, (timer, context) -> when(timer.get()).thenReturn(time))) {
      schedulers.when(CommandScheduler::getInstance).thenReturn(scheduler);
      Pose2d endPose = new Pose2d(1, 0, new Rotation2d());
      Path path =
          new Path(new Pose2d(), endPose, 1, 1, endRadius, new ChassisVelocities(0.4, 0, 0));
      Command marker = mock(Command.class);
      Plan plan = new Plan(200, path).addCommand(2, marker);
      ArrayList<ChassisVelocities> outputs = new ArrayList<>();
      Command follow = Planner.followPath(plan, () -> endPose, outputs::add);
      follow.initialize();
      for (int i = 0; i < 5 && !follow.isFinished(); i++) {
        follow.execute();
      }
      assertTrue(follow.isFinished());
      assertFalse(outputs.isEmpty(), "Final sample must reach the drive consumer");
      assertEquals(0.4, outputs.getLast().vx, 1e-9);
      verify(scheduler).schedule(marker);
      logger.verify(() -> Logger.recordOutput("GompeiPlanner/Target/AtFinalSample", true));
      follow.end(false);
      assertEquals(0, outputs.getLast().vx);
    }
  }
}
