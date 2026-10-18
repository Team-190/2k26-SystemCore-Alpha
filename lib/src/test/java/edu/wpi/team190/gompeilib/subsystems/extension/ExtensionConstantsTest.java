package edu.wpi.team190.gompeilib.subsystems.extension;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;

public class ExtensionConstantsTest extends GompeiLibTestBase {
  private static MotorConfig motor() {
    return MotorConfig.builder()
        .withCanId(1)
        .withBus(CANBusId.named("rio"))
        .withCurrentLimits(
            CurrentLimits.fromDoubles()
                .withSupplyCurrentLimit(40)
                .withStatorCurrentLimit(40)
                .build())
        .build();
  }

  private static ExtensionConstants.ExtensionParameters parameters() {
    return ExtensionConstants.ExtensionParameters.builder()
        .withCarriageGearbox(DCMotor.getKrakenX60(1))
        .withCarriageMass(Kilograms.of(5.0))
        .withMinLength(Meters.of(0.0))
        .withMaxLength(Meters.of(1.0))
        .withSimulateGravity(true)
        .build();
  }

  @Test
  public void testConstantsAndBuilder() {
    MotorConfig motor = motor();
    ExtensionConstants.ExtensionParameters parameters = parameters();
    LinearConstraints constraints = LinearConstraints.builder().build();

    ExtensionConstants constants =
        ExtensionConstants.builder()
            .withMotor(motor)
            .withDrumRadius(Meters.of(0.02))
            .withExtensionParameters(parameters)
            .withConstraints(constraints)
            .withVoltageOffsetStep(Volts.of(0.5))
            .withHeightOffsetStep(Meters.of(0.01))
            .build();

    assertSame(motor, constants.motor);
    assertEquals(Meters.of(0.02), constants.drumRadius);
    assertSame(parameters, constants.extensionParameters);
    assertSame(constraints, constants.constraints);
    assertEquals(Volts.of(0.5), constants.voltageOffsetStep);
    assertEquals(Meters.of(0.01), constants.heightOffsetStep);
    assertEquals(Volts.of(1.0), constants.zeroVoltage);
    assertEquals(Amps.of(-20.0), constants.zeroCurrentThreshold);
    assertEquals(Amps.of(5.0), constants.zeroCurrentEpsilon);

    assertEquals(Kilograms.of(5.0), parameters.carriageMass());
    assertEquals(Meters.of(1.0), parameters.maxLength());
    assertTrue(parameters.simulateGravity());
  }

  @Test
  public void shouldRequireEveryField() {
    assertThrows(NullPointerException.class, () -> ExtensionConstants.builder().build());
    assertThrows(
        NullPointerException.class, () -> ExtensionConstants.ExtensionParameters.builder().build());
  }
}
