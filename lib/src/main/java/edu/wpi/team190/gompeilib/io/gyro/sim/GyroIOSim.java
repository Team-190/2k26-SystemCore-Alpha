package edu.wpi.team190.gompeilib.io.gyro.sim;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLib;
import edu.wpi.team190.gompeilib.io.gyro.inputs.GyroTelemetry;
import edu.wpi.team190.gompeilib.io.gyro.interfaces.GyroIO;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

/**
 * A simulated gyro. Whatever drives the simulation sets the yaw rate; the gyro integrates it each
 * cycle and reports one odometry sample per cycle.
 */
public class GyroIOSim implements GyroIO {
  private double yawRadians = 0.0;
  private double yawRateRadiansPerSecond = 0.0;

  /**
   * Sets the yaw rate the gyro integrates.
   *
   * @param yawRate the yaw rate
   */
  public void setYawVelocity(AngularVelocity yawRate) {
    yawRateRadiansPerSecond = yawRate.in(RadiansPerSecond);
  }

  @Override
  public boolean isConnected() {
    return true;
  }

  @Override
  public void setYaw(Angle yaw) {
    yawRadians = yaw.in(Radians);
  }

  @Override
  public void updateInputs(GyroTelemetry telemetry) {
    yawRadians += yawRateRadiansPerSecond * GompeiLib.getLoopPeriod();

    telemetry.connected = true;
    telemetry.yawPosition = Radians.of(yawRadians);
    telemetry.yawVelocity = RadiansPerSecond.of(yawRateRadiansPerSecond);
    telemetry.deviceAngularVelocityZ = RadiansPerSecond.of(yawRateRadiansPerSecond);
    telemetry.accelerationZ = Gs.of(1.0);
    telemetry.gravityVectorZ = 1.0;
    telemetry.quaternionW = Math.cos(yawRadians / 2.0);
    telemetry.quaternionZ = Math.sin(yawRadians / 2.0);
    telemetry.supplyVoltage = Volts.of(RobotController.getBatteryVoltage());
    telemetry.odometryTimestamps = new double[] {RobotController.getTime() / 1e6};
    telemetry.odometryYawPositions = new Angle[] {Radians.of(yawRadians)};
  }
}
