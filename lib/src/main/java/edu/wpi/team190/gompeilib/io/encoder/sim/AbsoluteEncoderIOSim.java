package edu.wpi.team190.gompeilib.io.encoder.sim;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.io.encoder.config.AbsoluteEncoderConfig;
import edu.wpi.team190.gompeilib.io.encoder.config.SensorDirection;
import edu.wpi.team190.gompeilib.io.encoder.inputs.AbsoluteEncoderTelemetry;
import edu.wpi.team190.gompeilib.io.encoder.inputs.MagnetHealth;
import edu.wpi.team190.gompeilib.io.encoder.interfaces.AbsoluteEncoderIO;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Angle;

/**
 * A simulated absolute encoder reading a {@link MechanismSim}. It never drives the mechanism; a
 * simulated motor does.
 */
public class AbsoluteEncoderIOSim implements AbsoluteEncoderIO {
  private final AbsoluteEncoderConfig config;
  private final MechanismSim mechanism;
  private final double sensorRatio;
  private double positionOffsetRotations = 0.0;

  /**
   * Creates a simulated encoder.
   *
   * @param config the encoder's config
   * @param mechanism the mechanism it measures
   * @param sensorRatio encoder rotations per gearbox-output rotation
   */
  public AbsoluteEncoderIOSim(
      AbsoluteEncoderConfig config, MechanismSim mechanism, double sensorRatio) {
    this.config = config;
    this.mechanism = mechanism;
    this.sensorRatio = sensorRatio;
  }

  private double sign() {
    return config.direction == SensorDirection.CLOCKWISE_POSITIVE ? -1.0 : 1.0;
  }

  private double rawRotations() {
    return sign() * mechanism.position().in(Rotations) * sensorRatio;
  }

  @Override
  public boolean isConnected() {
    return true;
  }

  @Override
  public void setPosition(Angle position) {
    positionOffsetRotations = position.in(Rotations) - rawRotations();
  }

  @Override
  public void updateInputs(AbsoluteEncoderTelemetry telemetry) {
    double raw = rawRotations();
    double velocity = sign() * mechanism.velocity().in(RotationsPerSecond) * sensorRatio;
    double top = config.discontinuityPoint.in(Rotations);
    double absolute = raw + config.magnetOffset.in(Rotations);
    absolute = absolute - Math.floor(absolute - (top - 1.0));

    telemetry.connected = true;
    telemetry.absolutePosition = Rotations.of(absolute);
    telemetry.position = Rotations.of(raw + positionOffsetRotations);
    telemetry.positionSinceBoot = Rotations.of(raw);
    telemetry.velocity = RotationsPerSecond.of(velocity);
    telemetry.unfilteredVelocity = RotationsPerSecond.of(velocity);
    telemetry.magnetHealth = MagnetHealth.GREEN;
    telemetry.supplyVoltage = Volts.of(RobotController.getBatteryVoltage());
  }
}
