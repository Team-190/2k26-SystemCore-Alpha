package edu.wpi.team190.gompeilib.robot.mechanisms.arm;

import static edu.wpi.team190.gompeilib.math.CustomUnits.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.physics.Inertia;
import edu.wpi.team190.gompeilib.io.physics.PlantCharacterization;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.VoltageUnit;

public class ArmConstantsTest extends GompeiLibTestBase {
  private static CharacterizationConfig<VoltageUnit> sysId() {
    return CharacterizationConfig.voltage()
        .withRampRate(VoltsPerSecond.ofNative(1))
        .withStepOutput(Volts.of(3))
        .withQuasistaticTimeout(Seconds.of(3))
        .withDynamicTimeout(Seconds.of(3))
        .build();
  }

  @Test
  public void testConstantsAndBuilder() {
    MotorConfig motor =
        MotorConfig.builder()
            .withCanId(1)
            .withBus(CANBusId.named("rio"))
            .withCurrentLimits(
                CurrentLimits.fromDoubles()
                    .withSupplyCurrentLimit(40)
                    .withStatorCurrentLimit(40)
                    .build())
            .build();
    ArmConstants.ArmParameters parameters =
        ArmConstants.ArmParameters.builder()
            .withArmGearbox(DCMotor.getKrakenX60(1))
            .withMinAngle(Degrees.of(-90))
            .withMaxAngle(Degrees.of(90))
            .withPlant(
                PlantCharacterization.ofArm(
                        DCMotor.getKrakenX60(1),
                        50.0,
                        Kilograms.of(2.0),
                        Meters.of(0.5),
                        Inertia.atOutput(KilogramSquareMeters.of(0.1)))
                    .build())
            .build();
    AngularPositionConstraints constraints = AngularPositionConstraints.builder().build();
    CharacterizationConfig<VoltageUnit> sysId = sysId();

    ArmConstants constants =
        ArmConstants.builder()
            .withMotor(motor)
            .withArmParameters(parameters)
            .withConstraints(constraints)
            .withVoltageOffsetStep(Volts.of(0.5))
            .withCharacterization(sysId)
            .withPositionOffsetStep(Degrees.of(5))
            .build();

    assertSame(motor, constants.motor);
    assertSame(parameters, constants.armParameters);
    assertSame(constraints, constants.constraints);
    assertEquals(Volts.of(0.5), constants.voltageOffsetStep);
    assertEquals(Degrees.of(5), constants.positionOffsetStep);
    assertSame(sysId, constants.characterization);
    assertEquals(Degrees.of(-90), parameters.minAngle());
    assertEquals(Degrees.of(90), parameters.maxAngle());
    assertEquals(
        PlantCharacterization.ofArm(
                DCMotor.getKrakenX60(1),
                50.0,
                Kilograms.of(2.0),
                Meters.of(0.5),
                Inertia.atOutput(KilogramSquareMeters.of(0.1)))
            .build(),
        parameters.plant());
    assertEquals(
        edu.wpi.team190.gompeilib.io.motor.outputs.OutputType.VOLTAGE,
        constants.closedLoopOutputType);
    assertEquals(Amps.of(1.0), constants.currentOffsetStep);
    assertEquals(Value.of(0.05), constants.dutyCycleOffsetStep);

    ArmConstants custom =
        constants.toBuilder()
            .withClosedLoopOutputType(edu.wpi.team190.gompeilib.io.motor.outputs.OutputType.CURRENT)
            .withCurrentOffsetStep(Amps.of(2.0))
            .withDutyCycleOffsetStep(Value.of(0.1))
            .build();
    assertEquals(
        edu.wpi.team190.gompeilib.io.motor.outputs.OutputType.CURRENT, custom.closedLoopOutputType);
    assertEquals(Amps.of(2.0), custom.currentOffsetStep);
    assertEquals(Value.of(0.1), custom.dutyCycleOffsetStep);
  }

  @Test
  public void shouldRequireEveryField() {
    assertThrows(NullPointerException.class, () -> ArmConstants.builder().build());
    assertThrows(NullPointerException.class, () -> ArmConstants.ArmParameters.builder().build());
  }
}
