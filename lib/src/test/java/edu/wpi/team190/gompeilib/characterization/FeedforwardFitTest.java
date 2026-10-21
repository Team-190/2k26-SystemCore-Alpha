package edu.wpi.team190.gompeilib.characterization;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.characterization.FeedforwardFit.Coefficient;
import edu.wpi.team190.gompeilib.characterization.FeedforwardFit.Result;
import edu.wpi.team190.gompeilib.characterization.FeedforwardFit.Sample;
import edu.wpi.team190.gompeilib.io.motor.config.GravityType;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.Random;
import org.junit.jupiter.api.Test;

public class FeedforwardFitTest {
  private static final double EPSILON = 1e-9;

  /** Samples from a known plant over varied motion, with optional noise on the effort. */
  private static List<Sample> samples(
      int count,
      double kS,
      double kV,
      double kA,
      double kG,
      Optional<GravityType> gravity,
      double noise,
      double backEmfPerVelocity) {
    Random random = new Random(190);
    List<Sample> samples = new ArrayList<>();
    for (int i = 0; i < count; i++) {
      double velocity = 4 * Math.sin(i * 0.37) + (i % 2 == 0 ? 0.6 : -0.6);
      double acceleration = 6 * Math.cos(i * 0.91);
      double position = 1.2 * Math.sin(i * 0.13);
      double load =
          gravity.map(type -> type == GravityType.COSINE ? Math.cos(position) : 1.0).orElse(0.0);
      double effort =
          kS * Math.signum(velocity)
              + kV * velocity
              + kA * acceleration
              + kG * load
              + noise * random.nextGaussian();
      samples.add(
          new Sample(effort, position, velocity, acceleration, backEmfPerVelocity * velocity));
    }
    return samples;
  }

  @Test
  public void testRecoversAPlantWithoutGravity() {
    Result result =
        FeedforwardFit.fit(
            samples(200, 0.3, 0.2, 0.05, 0, Optional.empty(), 0, 0.1), Optional.empty());

    assertEquals(0.3, result.kS().value(), EPSILON);
    assertEquals(0.2, result.kV().value(), EPSILON);
    assertEquals(0.05, result.kA().value(), EPSILON);
    assertEquals(Coefficient.NONE, result.kG());
    assertEquals(Optional.empty(), result.gravity());
    assertEquals(1.0, result.rSquared(), EPSILON);
    assertEquals(0.0, result.rmse(), 1e-9);
    assertEquals(200, result.samples());
    assertEquals(0.1, result.motorModelKV().getAsDouble(), EPSILON);
  }

  @Test
  public void testRecoversConstantAndCosineGravity() {
    Optional<GravityType> constant = Optional.of(GravityType.CONSTANT);
    Result elevator =
        FeedforwardFit.fit(samples(200, 0.3, 0.2, 0.05, 0.8, constant, 0, 0.1), constant);
    assertEquals(0.8, elevator.kG().value(), EPSILON);
    assertEquals(0.3, elevator.kS().value(), EPSILON);

    Optional<GravityType> cosine = Optional.of(GravityType.COSINE);
    Result arm = FeedforwardFit.fit(samples(200, 0.3, 0.2, 0.05, 0.8, cosine, 0, 0.1), cosine);
    assertEquals(0.8, arm.kG().value(), EPSILON);
    assertEquals(cosine, arm.gravity());
  }

  @Test
  public void testNoisyEffortGivesErrorBars() {
    Result result =
        FeedforwardFit.fit(
            samples(500, 0.3, 0.2, 0.05, 0, Optional.empty(), 0.02, 0.1), Optional.empty());

    assertEquals(0.3, result.kS().value(), 0.01);
    assertEquals(0.2, result.kV().value(), 0.005);
    assertEquals(0.05, result.kA().value(), 0.002);
    assertTrue(result.kS().standardError() > 0);
    assertTrue(result.rSquared() < 1.0 && result.rSquared() > 0.99);
    assertEquals(0.02, result.rmse(), 0.005);
  }

  @Test
  public void testDropsSamplesAtRest() {
    List<Sample> samples =
        new ArrayList<>(samples(30, 0.3, 0.2, 0.05, 0, Optional.empty(), 0, 0.1));
    for (int i = 0; i < 50; i++) {
      samples.add(new Sample(5.0, 0, FeedforwardFit.MIN_VELOCITY / 2, 0, 0));
    }

    Result result = FeedforwardFit.fit(samples, Optional.empty());
    assertEquals(30, result.samples());
    assertEquals(0.3, result.kS().value(), EPSILON);
  }

  @Test
  public void shouldRejectTooFewMovingSamples() {
    List<Sample> samples = samples(19, 0.3, 0.2, 0.05, 0, Optional.empty(), 0, 0.1);
    IllegalArgumentException error =
        assertThrows(
            IllegalArgumentException.class, () -> FeedforwardFit.fit(samples, Optional.empty()));
    assertEquals(
        "Need at least 20 moving samples to fit, got 19: did the mechanism move?",
        error.getMessage());
    assertThrows(
        NullPointerException.class,
        () -> FeedforwardFit.fit(samples(30, 0.3, 0.2, 0.05, 0, Optional.empty(), 0, 0), null));
  }

  @Test
  public void shouldRejectSamplesThatCantSeparateTheTerms() {
    // No acceleration at all: kA is invisible
    List<Sample> steady = new ArrayList<>();
    for (int i = 0; i < 40; i++) {
      double velocity = i % 2 == 0 ? 1.0 + i : -1.0 - i;
      steady.add(new Sample(0.3 * Math.signum(velocity) + 0.2 * velocity, 0, velocity, 0, 0));
    }
    assertThrows(
        IllegalArgumentException.class, () -> FeedforwardFit.fit(steady, Optional.empty()));

    // Acceleration always equal to velocity: kV and kA are the same term
    List<Sample> tied = new ArrayList<>();
    for (int i = 0; i < 40; i++) {
      double velocity = (i % 2 == 0 ? 1 : -1) * (1.0 + i);
      tied.add(new Sample(velocity, 0, velocity, velocity, 0));
    }
    IllegalArgumentException error =
        assertThrows(
            IllegalArgumentException.class, () -> FeedforwardFit.fit(tied, Optional.empty()));
    assertTrue(error.getMessage().startsWith("The samples can't tell the terms apart"));
  }

  @Test
  public void testConstantEffortExplainsEverything() {
    // The effort never varies, so there's no variance to explain
    List<Sample> samples = new ArrayList<>();
    for (int i = 0; i < 40; i++) {
      double velocity = 2 * Math.sin(i * 0.37) + (i % 2 == 0 ? 0.6 : -0.6);
      samples.add(new Sample(1.0, 0, velocity, Math.cos(i * 0.91), Double.NaN));
    }
    Result result = FeedforwardFit.fit(samples, Optional.of(GravityType.CONSTANT));
    assertEquals(1.0, result.rSquared());
    assertEquals(1.0, result.kG().value(), EPSILON);
    assertTrue(result.motorModelKV().isEmpty());
  }

  @Test
  public void testCorrelatedTermsStillSeparate() {
    // Velocity mostly follows its sign, so kS and kV are strongly correlated
    List<Sample> samples = new ArrayList<>();
    for (int i = 0; i < 100; i++) {
      double sign = i % 2 == 0 ? 1 : -1;
      double velocity = sign * (2 + 0.3 * Math.sin(i * 0.7));
      // Acceleration follows the speed's ripple, not its sign, so elimination has to swap rows
      double acceleration = sign * 0.3 * Math.sin(i * 0.7) + 0.05 * Math.cos(i * 1.3);
      samples.add(
          new Sample(
              0.5 * sign + 0.3 * velocity + 0.1 * acceleration,
              0,
              velocity,
              acceleration,
              i < 50 ? 0.2 * velocity : Double.NaN));
    }
    Result result = FeedforwardFit.fit(samples, Optional.empty());
    assertEquals(0.5, result.kS().value(), 1e-6);
    assertEquals(0.3, result.kV().value(), 1e-6);
    assertEquals(0.1, result.kA().value(), 1e-6);
    assertEquals(0.2, result.motorModelKV().getAsDouble(), EPSILON);
  }

  @Test
  public void testRelativeError() {
    assertEquals(0.5, new Coefficient(-2, 1).relativeError(), EPSILON);
    assertEquals(0.0, new Coefficient(0, 1).relativeError());
    assertEquals(0.0, Coefficient.NONE.relativeError());
  }

  private static Result result(
      Coefficient kS,
      Coefficient kV,
      Coefficient kA,
      Coefficient kG,
      Optional<GravityType> gravity,
      OptionalDouble motorModelKV) {
    return new Result(kS, kV, kA, kG, gravity, 0.99, 0.1, 100, motorModelKV);
  }

  @Test
  public void testWarnsAboutNegativeAndUncertainTerms() {
    Coefficient good = new Coefficient(1.0, 0.01);
    Result result =
        result(
            new Coefficient(-0.1, 0.001),
            new Coefficient(0.2, 0.1),
            good,
            new Coefficient(-1.0, 0.5),
            Optional.of(GravityType.COSINE),
            OptionalDouble.empty());

    assertEquals(
        List.of(
            "kS is negative",
            "kV is uncertain (±50%): the legs didn't separate it from the other terms",
            "kG is negative",
            "kG is uncertain (±50%): the legs didn't separate it from the other terms"),
        result.warnings(true));

    Result clean =
        result(good, good, good, new Coefficient(-1, 1), Optional.empty(), OptionalDouble.empty());
    assertTrue(clean.warnings(true).isEmpty());
  }

  @Test
  public void testWarnsWhenAVoltageKVIsFarFromTheMotorModel() {
    Coefficient good = new Coefficient(1.0, 0.01);
    Result high =
        result(
            good,
            new Coefficient(4.0, 0.01),
            good,
            Coefficient.NONE,
            Optional.empty(),
            OptionalDouble.of(1.0));
    assertEquals(
        List.of(
            "kV is 4.00x the motor model's 1.00000 V/(rad/s): the legs may not have reached a"
                + " steady speed, so consider the motor model's kV"),
        high.warnings(true));
    assertTrue(high.warnings(false).isEmpty());

    Result low =
        result(
            good,
            new Coefficient(0.25, 0.001),
            good,
            Coefficient.NONE,
            Optional.empty(),
            OptionalDouble.of(1.0));
    assertEquals(1, low.warnings(true).size());

    Result close =
        result(
            good,
            new Coefficient(1.5, 0.01),
            good,
            Coefficient.NONE,
            Optional.empty(),
            OptionalDouble.of(1.0));
    assertTrue(close.warnings(true).isEmpty());
  }
}
