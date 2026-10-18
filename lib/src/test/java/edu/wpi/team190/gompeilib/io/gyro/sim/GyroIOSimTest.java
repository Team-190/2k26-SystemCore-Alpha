package edu.wpi.team190.gompeilib.io.gyro.sim;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.gyro.inputs.GyroTelemetryAutoLogged;
import org.junit.jupiter.api.Test;

public class GyroIOSimTest extends GompeiLibTestBase {

  @Test
  public void testIntegratesTheYawRate() {
    GyroIOSim gyro = new GyroIOSim();
    GyroTelemetryAutoLogged telemetry = new GyroTelemetryAutoLogged();
    gyro.setYawVelocity(RadiansPerSecond.of(1.0));

    for (int i = 0; i < 50; i++) {
      gyro.updateInputs(telemetry);
    }

    assertTrue(gyro.isConnected());
    assertTrue(telemetry.connected);
    assertEquals(1.0, telemetry.yawPosition.in(Radians), 1e-9);
    assertEquals(1.0, telemetry.yawVelocity.in(RadiansPerSecond), 1e-9);
    assertEquals(telemetry.yawVelocity, telemetry.deviceAngularVelocityZ);
    assertEquals(1.0, telemetry.accelerationZ.in(Gs), 1e-9);
    assertEquals(Math.cos(0.5), telemetry.quaternionW, 1e-9);
    assertEquals(Math.sin(0.5), telemetry.quaternionZ, 1e-9);
    assertEquals(1, telemetry.odometryTimestamps.length);
    assertEquals(telemetry.yawPosition, telemetry.odometryYawPositions[0]);
  }

  @Test
  public void testSetYaw() {
    GyroIOSim gyro = new GyroIOSim();
    GyroTelemetryAutoLogged telemetry = new GyroTelemetryAutoLogged();

    gyro.setYaw(Degrees.of(90.0));
    gyro.updateInputs(telemetry);

    assertEquals(90.0, telemetry.yawPosition.in(Degrees), 1e-9);
  }
}
