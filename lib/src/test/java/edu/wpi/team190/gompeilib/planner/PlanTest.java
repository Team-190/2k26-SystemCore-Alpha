package edu.wpi.team190.gompeilib.planner;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import edu.wpi.team190.gompeilib.core.GompeiLib;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.mockito.MockedStatic;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

class PlanTest {
  static Path straightPath() {
    // Two-second triangular profile: accelerate for one second, then decelerate.
    return new Path(new Pose2d(), new Pose2d(1, 0, new Rotation2d()), 1, 1);
  }

  @ParameterizedTest
  @ValueSource(doubles = {0, -1, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
  void rejectsInvalidExplicitRates(double rate) {
    assertThrows(IllegalArgumentException.class, () -> new Plan(rate, straightPath()));
    assertThrows(IllegalArgumentException.class, () -> new Plan(rate));
  }

  @Test
  void rejectsSampleCountOverflowBeforeAllocating() {
    assertThrows(IllegalArgumentException.class, () -> new Plan(Double.MAX_VALUE, straightPath()));
  }

  @Test
  void capturesConfiguredRateForGenerationAndPlayback() {
    try (MockedStatic<GompeiLib> gompei = mockStatic(GompeiLib.class)) {
      gompei.when(GompeiLib::getLoopPeriod).thenReturn(0.005);
      Plan plan = new Plan(straightPath());
      gompei.when(GompeiLib::getLoopPeriod).thenReturn(0.02);
      assertEquals(200, plan.getSampleRateHz());
      assertEquals(401, plan.getPlan().size());
      assertEquals(0.5, plan.sample(1).pose().getX(), 1e-9);
      assertEquals(1, plan.speedAt(1), 1e-9);
      assertEquals(2, plan.getPathEndTime(0), 1e-9);
      gompei.verify(GompeiLib::getLoopPeriod, times(1));
    }
  }

  @Test
  void fallbackStaysCapturedAfterInitializationAndDoesNotSuppressOtherWarnings() {
    try (MockedStatic<GompeiLib> gompei = mockStatic(GompeiLib.class);
        MockedStatic<DriverStationErrors> errors = mockStatic(DriverStationErrors.class)) {
      gompei.when(GompeiLib::getLoopPeriod).thenThrow(new IllegalStateException());
      Plan fallback = new Plan(straightPath());
      gompei.when(GompeiLib::getLoopPeriod).thenReturn(0.0);
      assertEquals(200, new Plan(straightPath()).getSampleRateHz());
      errors.verify(
          () -> DriverStationErrors.reportWarning(contains("not initialized"), eq(false)));
      errors.verify(() -> DriverStationErrors.reportWarning(contains("Invalid"), eq(false)));
      gompei.when(GompeiLib::getLoopPeriod).thenReturn(0.02);
      assertEquals(50, new Plan(straightPath()).getSampleRateHz());
      assertEquals(200, fallback.getSampleRateHz());
      assertEquals(0.5, fallback.sample(1).pose().getX(), 1e-9);
    }
  }

  @ParameterizedTest
  @ValueSource(
      doubles = {
        0,
        -1,
        Double.NaN,
        Double.POSITIVE_INFINITY,
        Double.NEGATIVE_INFINITY,
        Double.MIN_VALUE
      })
  void invalidLoopPeriodsWarnOnEveryPlanConstruction(double period) {
    try (MockedStatic<GompeiLib> gompei = mockStatic(GompeiLib.class);
        MockedStatic<DriverStationErrors> errors = mockStatic(DriverStationErrors.class)) {
      gompei.when(GompeiLib::getLoopPeriod).thenReturn(period);
      assertEquals(200, new Plan(straightPath()).getSampleRateHz());
      assertEquals(200, new Plan(straightPath()).getSampleRateHz());
      errors.verify(
          () -> DriverStationErrors.reportWarning(contains("Invalid"), eq(false)), times(2));
    }
  }

  @Test
  void interpolatesPoseVelocityAndDistanceBetweenSamples() {
    Plan plan = new Plan(10, straightPath());
    State sample = plan.sample(0.15);
    assertEquals(0.0125, sample.pose().getX(), 1e-9);
    assertEquals(0.0125, sample.distance(), 1e-9);
    assertEquals(0.15, sample.velocity().vx, 1e-9);
    assertEquals(0.15, plan.speedAt(0.15), 1e-9);
    assertEquals(0.001, plan.speedAt(0.001), 1e-9);
  }

  @Test
  void clampsNegativeFinalAndOvershootingTimesBeforeIndexing() {
    Plan plan = new Plan(200, straightPath());
    for (double time : new double[] {-1, -Double.MAX_VALUE, Double.NEGATIVE_INFINITY}) {
      assertEquals(plan.getPlan().getFirst(), plan.sample(time));
      assertEquals(0, plan.speedAt(time));
    }
    for (double time : new double[] {2, 2.005, Double.MAX_VALUE, Double.POSITIVE_INFINITY}) {
      assertSame(plan.getPlan().getLast(), plan.sample(time));
      assertEquals(0, plan.speedAt(time));
    }
    assertEquals(0.005, plan.speedAt(1.995), 1e-9);
    assertEquals(0.0025, plan.speedAt(1.9975), 1e-9);
    assertThrows(IllegalArgumentException.class, () -> plan.sample(Double.NaN));
    assertThrows(IllegalArgumentException.class, () -> plan.speedAt(Double.NaN));
  }

  @Test
  void emptyPlanHasAnExplicitSamplingError() {
    Plan plan = new Plan(200);
    assertTrue(plan.getPlan().isEmpty());
    assertThrows(IllegalStateException.class, () -> plan.sample(0));
    assertThrows(IllegalStateException.class, () -> plan.speedAt(0));
  }
}
