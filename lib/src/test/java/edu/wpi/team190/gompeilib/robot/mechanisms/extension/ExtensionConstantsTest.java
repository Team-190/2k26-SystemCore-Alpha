package edu.wpi.team190.gompeilib.robot.mechanisms.extension;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig;
import edu.wpi.team190.gompeilib.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.physics.PlantCharacterization;
import edu.wpi.team190.gompeilib.math.CustomUnits;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.VoltageUnit;

public class ExtensionConstantsTest extends GompeiLibTestBase {
  private static CharacterizationConfig<VoltageUnit> sysId() {
    return CharacterizationConfig.voltage()
        .withRampRate(CustomUnits.voltsPerSecond.ofNative(1))
        .withStepOutput(Volts.of(3))
        .withQuasistaticTimeout(Seconds.of(3))
        .withDynamicTimeout(Seconds.of(3))
        .build();
  }

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
        .withMinLength(Meters.of(0.0))
        .withMaxLength(Meters.of(1.0))
        .withPlant(
            PlantCharacterization.ofElevator(
                    DCMotor.getNeo550(1), 10.0, Kilograms.of(5.0), Meters.of(0.02))
                .build())
        .build();
  }

  @Test
  public void testConstantsAndBuilder() {
    MotorConfig motor = motor();
    ExtensionConstants.ExtensionParameters parameters = parameters();
    LinearConstraints constraints = LinearConstraints.builder().build();
    CharacterizationConfig<VoltageUnit> sysId = sysId();

    ExtensionConstants constants =
        ExtensionConstants.builder()
            .withMotor(motor)
            .withDrumRadius(Meters.of(0.02))
            .withExtensionParameters(parameters)
            .withConstraints(constraints)
            .withVoltageOffsetStep(Volts.of(0.5))
            .withCharacterization(sysId)
            .withHeightOffsetStep(Meters.of(0.01))
            .build();

    assertSame(motor, constants.motor);
    assertEquals(Meters.of(0.02), constants.drumRadius);
    assertSame(parameters, constants.extensionParameters);
    assertSame(constraints, constants.constraints);
    assertEquals(Volts.of(0.5), constants.voltageOffsetStep);
    assertEquals(Meters.of(0.01), constants.heightOffsetStep);
    assertSame(sysId, constants.characterization);
    assertTrue(constants.homing.isEmpty());

    assertEquals(
        PlantCharacterization.ofElevator(
                DCMotor.getNeo550(1), 10.0, Kilograms.of(5.0), Meters.of(0.02))
            .build(),
        parameters.plant());
    assertEquals(Meters.of(1.0), parameters.maxLength());
    assertEquals(
        edu.wpi.team190.gompeilib.io.motor.outputs.OutputType.VOLTAGE,
        constants.closedLoopOutputType);
    assertEquals(Amps.of(1.0), constants.currentOffsetStep);
    assertEquals(Value.of(0.05), constants.dutyCycleOffsetStep);

    ExtensionConstants custom =
        constants.toBuilder()
            .withClosedLoopOutputType(edu.wpi.team190.gompeilib.io.motor.outputs.OutputType.CURRENT)
            .withCurrentOffsetStep(Amps.of(3.0))
            .withDutyCycleOffsetStep(Value.of(0.1))
            .build();
    assertEquals(
        edu.wpi.team190.gompeilib.io.motor.outputs.OutputType.CURRENT, custom.closedLoopOutputType);
    assertEquals(Amps.of(3.0), custom.currentOffsetStep);
    assertEquals(Value.of(0.1), custom.dutyCycleOffsetStep);
  }

  @Test
  public void shouldRequireEveryField() {
    assertThrows(NullPointerException.class, () -> ExtensionConstants.builder().build());
    assertThrows(
        NullPointerException.class, () -> ExtensionConstants.ExtensionParameters.builder().build());
  }
}
