package edu.wpi.team190.gompeilib.io.encoder.sim;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.encoder.config.AbsoluteEncoderConfig;
import edu.wpi.team190.gompeilib.io.encoder.config.SensorDirection;
import edu.wpi.team190.gompeilib.io.encoder.inputs.AbsoluteEncoderTelemetryAutoLogged;
import edu.wpi.team190.gompeilib.io.encoder.inputs.MagnetHealth;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import edu.wpi.team190.gompeilib.io.physics.PlantCharacterization;
import edu.wpi.team190.gompeilib.io.physics.PlantMechanismSim;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;

public class AbsoluteEncoderIOSimTest extends GompeiLibTestBase {
  private MechanismSim mechanism;
  private AbsoluteEncoderTelemetryAutoLogged telemetry;

  @BeforeEach
  public void setUp() {
    mechanism =
        new PlantMechanismSim(
            DCMotor.getKrakenX60(1),
            1.0,
            PlantCharacterization.ofInertia(
                    DCMotor.getKrakenX60(1), 1.0, KilogramSquareMeters.of(0.001))
                .build());
    telemetry = new AbsoluteEncoderTelemetryAutoLogged();
  }

  private static AbsoluteEncoderConfig.AbsoluteEncoderConfigBuilder config() {
    return AbsoluteEncoderConfig.builder().withCanId(1).withBus(CANBusId.named("rio"));
  }

  /** Spins the mechanism forward for a while, then returns its position in rotations. */
  private double spin() {
    mechanism.setInputVoltage(Volts.of(6.0));
    for (int i = 0; i < 10; i++) {
      mechanism.update(Seconds.of(0.02));
    }
    return mechanism.position().in(Rotations);
  }

  @Test
  public void testReadsTheMechanismThroughTheSensorRatio() {
    AbsoluteEncoderIOSim encoder = new AbsoluteEncoderIOSim(config().build(), mechanism, 2.0);
    double mechanismRotations = spin();

    encoder.updateInputs(telemetry);

    assertTrue(encoder.isConnected());
    assertTrue(telemetry.connected);
    assertEquals(2.0 * mechanismRotations, telemetry.position.in(Rotations), 1e-9);
    assertEquals(telemetry.position, telemetry.positionSinceBoot);
    assertEquals(
        2.0 * mechanism.velocity().in(RotationsPerSecond),
        telemetry.velocity.in(RotationsPerSecond),
        1e-9);
    assertEquals(telemetry.velocity, telemetry.unfilteredVelocity);
    assertEquals(MagnetHealth.GREEN, telemetry.magnetHealth);
    assertTrue(telemetry.supplyVoltage.in(Volts) > 0);
  }

  @Test
  public void testAbsolutePositionWrapsAtTheDiscontinuity() {
    AbsoluteEncoderIOSim encoder =
        new AbsoluteEncoderIOSim(
            config()
                .withMagnetOffset(Rotations.of(0.25))
                .withDiscontinuityPoint(Rotations.of(1.0))
                .build(),
            mechanism,
            10.0);
    double raw = 10.0 * spin() + 0.25;

    encoder.updateInputs(telemetry);

    double absolute = telemetry.absolutePosition.in(Rotations);
    assertTrue(absolute >= 0.0 && absolute < 1.0, "absolute " + absolute);
    assertEquals(raw - Math.floor(raw), absolute, 1e-9);
  }

  @Test
  public void testDefaultRangeIsCenteredOnZero() {
    AbsoluteEncoderIOSim encoder = new AbsoluteEncoderIOSim(config().build(), mechanism, 10.0);
    spin();

    encoder.updateInputs(telemetry);

    double absolute = telemetry.absolutePosition.in(Rotations);
    assertTrue(absolute >= -0.5 && absolute < 0.5, "absolute " + absolute);
  }

  @Test
  public void testClockwiseDirectionNegatesReadings() {
    AbsoluteEncoderIOSim encoder =
        new AbsoluteEncoderIOSim(
            config().withDirection(SensorDirection.CLOCKWISE_POSITIVE).build(), mechanism, 1.0);
    double mechanismRotations = spin();

    encoder.updateInputs(telemetry);

    assertEquals(-mechanismRotations, telemetry.position.in(Rotations), 1e-9);
    assertTrue(telemetry.velocity.in(RotationsPerSecond) < 0);
  }

  @Test
  public void testSetPositionOffsetsOnlyTheRelativePosition() {
    AbsoluteEncoderIOSim encoder = new AbsoluteEncoderIOSim(config().build(), mechanism, 1.0);
    double mechanismRotations = spin();

    encoder.setPosition(Rotations.of(5.0));
    encoder.updateInputs(telemetry);

    assertEquals(5.0, telemetry.position.in(Rotations), 1e-9);
    assertEquals(mechanismRotations, telemetry.positionSinceBoot.in(Rotations), 1e-9);
  }
}
