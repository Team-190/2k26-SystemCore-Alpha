package edu.wpi.team190.gompeilib.characterization;

import edu.wpi.team190.gompeilib.io.motor.config.GravityType;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalDouble;

/**
 * Fits a mechanism's feedforward model to samples taken while it is characterized, by ordinary
 * least squares over every leg together:
 *
 * <pre>u = kS·sign(ω) + kV·ω + kA·α [+ kG·g(θ)]</pre>
 *
 * <p>All values are at the gearbox output in radians, and the effort is in base units (volts or
 * amps). Samples slower than {@link #MIN_VELOCITY} are dropped, since static friction's direction
 * is undefined at rest.
 */
public final class FeedforwardFit {
  /** Samples slower than this, in radians per second, are dropped. */
  public static final double MIN_VELOCITY = 0.01;

  /** The fewest moving samples a fit accepts. */
  public static final int MIN_SAMPLES = 20;

  /** A coefficient's standard error above this fraction of its value earns a warning. */
  public static final double UNCERTAIN_FRACTION = 0.2;

  /** A voltage kV off from the motor model by more than this factor earns a warning. */
  public static final double KV_MISMATCH_FACTOR = 3.0;

  /** Normalized pivots below this mean two terms can't be told apart. */
  private static final double SINGULAR_PIVOT = 1e-9;

  private FeedforwardFit() {}

  /**
   * One instant of a characterization.
   *
   * @param effort the effort the motors applied, in base units
   * @param position the position, in radians
   * @param velocity the velocity, in radians per second
   * @param acceleration the acceleration, in radians per second squared
   * @param backEmf the voltage the motor model says the motors' speed generates, or NaN if unknown
   */
  public record Sample(
      double effort, double position, double velocity, double acceleration, double backEmf) {}

  /**
   * A fitted coefficient.
   *
   * @param value the coefficient
   * @param standardError its standard error
   */
  public record Coefficient(double value, double standardError) {
    /** A term the fit left out. */
    public static final Coefficient NONE = new Coefficient(0.0, 0.0);

    /**
     * Returns the standard error as a fraction of the value.
     *
     * @return the relative error, or zero for a zero coefficient
     */
    public double relativeError() {
      return value == 0.0 ? 0.0 : Math.abs(standardError / value);
    }
  }

  /**
   * A fit.
   *
   * @param kS static friction
   * @param kV effort per radian per second
   * @param kA effort per radian per second squared
   * @param kG effort against gravity at full load, or {@link Coefficient#NONE} without gravity
   * @param gravity how gravity loads the mechanism, or empty if it doesn't
   * @param rSquared the fraction of the effort's variance the model explains
   * @param rmse the root-mean-square residual, in base effort units
   * @param samples how many samples were fitted
   * @param motorModelKV the kV the motor model predicts from back-EMF alone, if known
   */
  public record Result(
      Coefficient kS,
      Coefficient kV,
      Coefficient kA,
      Coefficient kG,
      Optional<GravityType> gravity,
      double rSquared,
      double rmse,
      int samples,
      OptionalDouble motorModelKV) {

    /**
     * Returns what looks wrong with the fit: negative or uncertain terms, and for a voltage fit, a
     * kV far from what the motor's back-EMF alone predicts.
     *
     * @param voltage whether the effort is voltage
     * @return the warnings, empty if none
     */
    public List<String> warnings(boolean voltage) {
      List<String> warnings = new ArrayList<>();
      check(warnings, "kS", kS);
      check(warnings, "kV", kV);
      check(warnings, "kA", kA);
      if (gravity.isPresent()) {
        check(warnings, "kG", kG);
      }
      if (voltage && motorModelKV.isPresent()) {
        double model = motorModelKV.getAsDouble();
        double ratio = kV.value() / model;
        if (ratio > KV_MISMATCH_FACTOR || ratio < 1.0 / KV_MISMATCH_FACTOR) {
          warnings.add(
              String.format(
                  "kV is %.2fx the motor model's %.6g V/(rad/s): the legs may not have reached a"
                      + " steady speed, so consider the motor model's kV",
                  ratio, model));
        }
      }
      return warnings;
    }

    private static void check(List<String> warnings, String name, Coefficient coefficient) {
      if (coefficient.value() < 0) {
        warnings.add(name + " is negative");
      }
      if (coefficient.relativeError() > UNCERTAIN_FRACTION) {
        warnings.add(
            String.format(
                "%s is uncertain (±%.0f%%): the legs didn't separate it from the other terms",
                name, 100 * coefficient.relativeError()));
      }
    }
  }

  /**
   * Fits the samples.
   *
   * @param samples the samples, in any order
   * @param gravity how gravity loads the mechanism, or empty to leave kG out
   * @return the fit
   * @throws IllegalArgumentException if fewer than {@link #MIN_SAMPLES} samples are moving, or the
   *     samples can't tell two terms apart
   */
  public static Result fit(List<Sample> samples, Optional<GravityType> gravity) {
    Objects.requireNonNull(gravity, "gravity");
    List<Sample> moving =
        samples.stream().filter(sample -> Math.abs(sample.velocity()) >= MIN_VELOCITY).toList();
    if (moving.size() < MIN_SAMPLES) {
      throw new IllegalArgumentException(
          "Need at least "
              + MIN_SAMPLES
              + " moving samples to fit, got "
              + moving.size()
              + ": did the mechanism move?");
    }

    int terms = gravity.isPresent() ? 4 : 3;
    double[][] normal = new double[terms][terms];
    double[] projection = new double[terms];
    double effortSum = 0.0;
    for (Sample sample : moving) {
      double[] row = features(sample, gravity, terms);
      for (int i = 0; i < terms; i++) {
        projection[i] += row[i] * sample.effort();
        for (int j = 0; j < terms; j++) {
          normal[i][j] += row[i] * row[j];
        }
      }
      effortSum += sample.effort();
    }

    double[][] inverse = invert(normal);
    double[] beta = new double[terms];
    for (int i = 0; i < terms; i++) {
      for (int j = 0; j < terms; j++) {
        beta[i] += inverse[i][j] * projection[j];
      }
    }

    double mean = effortSum / moving.size();
    double residualSquares = 0.0;
    double totalSquares = 0.0;
    double backEmfDotVelocity = 0.0;
    double velocitySquares = 0.0;
    for (Sample sample : moving) {
      double[] row = features(sample, gravity, terms);
      double predicted = 0.0;
      for (int i = 0; i < terms; i++) {
        predicted += beta[i] * row[i];
      }
      residualSquares += Math.pow(sample.effort() - predicted, 2);
      totalSquares += Math.pow(sample.effort() - mean, 2);
      if (Double.isFinite(sample.backEmf())) {
        backEmfDotVelocity += sample.backEmf() * sample.velocity();
        velocitySquares += sample.velocity() * sample.velocity();
      }
    }

    double variance = residualSquares / (moving.size() - terms);
    Coefficient[] coefficients = new Coefficient[terms];
    for (int i = 0; i < terms; i++) {
      coefficients[i] = new Coefficient(beta[i], Math.sqrt(variance * inverse[i][i]));
    }
    return new Result(
        coefficients[0],
        coefficients[1],
        coefficients[2],
        gravity.isPresent() ? coefficients[3] : Coefficient.NONE,
        gravity,
        totalSquares > 0 ? 1.0 - residualSquares / totalSquares : 1.0,
        Math.sqrt(residualSquares / moving.size()),
        moving.size(),
        velocitySquares > 0
            ? OptionalDouble.of(backEmfDotVelocity / velocitySquares)
            : OptionalDouble.empty());
  }

  private static double[] features(Sample sample, Optional<GravityType> gravity, int terms) {
    double[] row = new double[terms];
    row[0] = Math.signum(sample.velocity());
    row[1] = sample.velocity();
    row[2] = sample.acceleration();
    if (gravity.isPresent()) {
      row[3] = gravity.get() == GravityType.COSINE ? Math.cos(sample.position()) : 1.0;
    }
    return row;
  }

  /**
   * Inverts a symmetric positive semi-definite matrix by Gauss-Jordan elimination, scaled to unit
   * diagonal first so terms in very different units compare fairly.
   */
  private static double[][] invert(double[][] matrix) {
    int n = matrix.length;
    double[] scale = new double[n];
    for (int i = 0; i < n; i++) {
      scale[i] = Math.sqrt(matrix[i][i]);
      if (scale[i] == 0) {
        throw singular();
      }
    }
    double[][] work = new double[n][2 * n];
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        work[i][j] = matrix[i][j] / (scale[i] * scale[j]);
      }
      work[i][n + i] = 1.0;
    }
    for (int column = 0; column < n; column++) {
      int pivot = column;
      for (int row = column + 1; row < n; row++) {
        if (Math.abs(work[row][column]) > Math.abs(work[pivot][column])) {
          pivot = row;
        }
      }
      if (Math.abs(work[pivot][column]) < SINGULAR_PIVOT) {
        throw singular();
      }
      double[] swap = work[column];
      work[column] = work[pivot];
      work[pivot] = swap;
      double divisor = work[column][column];
      for (int j = 0; j < 2 * n; j++) {
        work[column][j] /= divisor;
      }
      for (int row = 0; row < n; row++) {
        if (row != column) {
          double factor = work[row][column];
          for (int j = 0; j < 2 * n; j++) {
            work[row][j] -= factor * work[column][j];
          }
        }
      }
    }
    double[][] inverse = new double[n][n];
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        inverse[i][j] = work[i][n + j] / (scale[i] * scale[j]);
      }
    }
    return inverse;
  }

  private static IllegalArgumentException singular() {
    return new IllegalArgumentException(
        "The samples can't tell the terms apart: run both quasistatic and dynamic legs in both"
            + " directions");
  }
}
