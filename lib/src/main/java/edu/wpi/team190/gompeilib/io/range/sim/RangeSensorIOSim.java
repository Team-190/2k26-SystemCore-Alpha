package edu.wpi.team190.gompeilib.io.range.sim;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.io.range.config.RangeSensorConfig;
import edu.wpi.team190.gompeilib.io.range.config.RangeUpdateMode;
import edu.wpi.team190.gompeilib.io.range.inputs.MeasurementHealth;
import edu.wpi.team190.gompeilib.io.range.inputs.RangeSensorTelemetry;
import edu.wpi.team190.gompeilib.io.range.interfaces.RangeSensorIO;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Distance;

/**
 * A simulated range sensor. Whatever drives the simulation sets the distance to the nearest target;
 * detection follows the configured threshold and hysteresis like the hardware.
 */
public class RangeSensorIOSim implements RangeSensorIO {
  /** The signal strength reported for a target. */
  public static final double SIGNAL_STRENGTH = 10000.0;

  private final RangeSensorConfig config;
  private double distanceMeters;
  private boolean detected = false;

  /**
   * Creates a simulated range sensor with nothing in view.
   *
   * @param config the sensor's config
   */
  public RangeSensorIOSim(RangeSensorConfig config) {
    this.config = config;
    this.distanceMeters = Double.POSITIVE_INFINITY;
  }

  /**
   * Sets the distance to the nearest target.
   *
   * @param distance the distance, or null for nothing in view
   */
  public void setDistance(Distance distance) {
    distanceMeters = distance == null ? Double.POSITIVE_INFINITY : distance.in(Meters);
  }

  @Override
  public boolean isConnected() {
    return true;
  }

  @Override
  public void updateInputs(RangeSensorTelemetry telemetry) {
    double threshold = config.proximityThreshold.in(Meters);
    double hysteresis = config.proximityHysteresis.in(Meters);
    detected = detected ? distanceMeters <= threshold + hysteresis : distanceMeters <= threshold;
    boolean inView = Double.isFinite(distanceMeters);

    telemetry.connected = true;
    telemetry.distance = Meters.of(inView ? distanceMeters : 0.0);
    telemetry.detected = detected;
    telemetry.signalStrength = inView ? SIGNAL_STRENGTH : 0.0;
    telemetry.measurementHealth = inView ? MeasurementHealth.GOOD : MeasurementHealth.BAD;
    telemetry.measurementTime =
        Seconds.of(
            1.0
                / (config.updateMode == RangeUpdateMode.SHORT_RANGE_100HZ
                    ? 100.0
                    : config.updateFrequency.in(Hertz)));
    telemetry.fovCenterX = config.fovCenterX;
    telemetry.fovCenterY = config.fovCenterY;
    telemetry.fovRangeX = config.fovRangeX;
    telemetry.fovRangeY = config.fovRangeY;
    telemetry.supplyVoltage = Volts.of(RobotController.getBatteryVoltage());
  }
}
