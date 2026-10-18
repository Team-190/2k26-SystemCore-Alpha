package edu.wpi.team190.gompeilib.io.encoder.phoenix;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.team190.gompeilib.io.encoder.config.AbsoluteEncoderConfig;
import edu.wpi.team190.gompeilib.io.encoder.config.SensorDirection;
import edu.wpi.team190.gompeilib.io.encoder.inputs.AbsoluteEncoderTelemetry;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import edu.wpi.team190.gompeilib.logging.Trace;
import org.wpilib.system.RobotController;

/**
 * A CANcoder running in Phoenix6 simulation, its magnet turned by a {@link MechanismSim}. A TalonFX
 * that uses it as a remote sensor reads the same simulated device.
 */
public class AbsoluteEncoderIOCANcoderSim extends AbsoluteEncoderIOCANcoder {
  private final MechanismSim mechanism;
  private final double sensorRatio;
  private final CANcoderSimState simState;

  /**
   * Creates a simulated CANcoder.
   *
   * @param config the encoder's config
   * @param mechanism the mechanism it measures
   * @param sensorRatio encoder rotations per gearbox-output rotation
   */
  public AbsoluteEncoderIOCANcoderSim(
      AbsoluteEncoderConfig config, MechanismSim mechanism, double sensorRatio) {
    super(config);
    this.mechanism = mechanism;
    this.sensorRatio = sensorRatio;
    simState = cancoder.getSimState();
    simState.Orientation =
        config.direction == SensorDirection.CLOCKWISE_POSITIVE
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
  }

  @Override
  @Trace
  public void updateInputs(AbsoluteEncoderTelemetry telemetry) {
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    simState.setRawPosition(mechanism.position().times(sensorRatio));
    simState.setVelocity(mechanism.velocity().times(sensorRatio));
    super.updateInputs(telemetry);
  }
}
