package edu.wpi.team190.gompeilib.robot.mechanisms.flywheel;

import static edu.wpi.team190.gompeilib.math.CustomUnits.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig;
import edu.wpi.team190.gompeilib.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.physics.PlantCharacterization;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.VoltageUnit;

public class GenericFlywheelConstantsTest extends GompeiLibTestBase {
  @Test
  public void testConstantsAndBuilder() {
    MotorConfig motor =
        MotorConfig.builder()
            .withCanId(1)
            .withBus(CANBusId.named("rio"))
            .withCurrentLimits(
                CurrentLimits.fromDoubles()
                    .withSupplyCurrentLimit(40)
                    .withStatorCurrentLimit(60)
                    .build())
            .build();
    DCMotor gearbox = DCMotor.getKrakenX60Foc(2);
    AngularVelocityConstraints constraints = AngularVelocityConstraints.builder().build();
    CharacterizationConfig<VoltageUnit> voltageCharacterization =
        CharacterizationConfig.voltage()
            .withRampRate(VoltsPerSecond.ofNative(1))
            .withStepOutput(Volts.of(3))
            .withQuasistaticTimeout(Seconds.of(3))
            .withDynamicTimeout(Seconds.of(3))
            .build();
    CharacterizationConfig<CurrentUnit> currentCharacterization =
        CharacterizationConfig.current()
            .withRampRate(AmpsPerSecond.ofNative(1))
            .withStepOutput(Amps.of(10))
            .withQuasistaticTimeout(Seconds.of(5))
            .withDynamicTimeout(Seconds.of(5))
            .build();

    GenericFlywheelConstants constants =
        GenericFlywheelConstants.builder()
            .withMotor(motor)
            .withFlywheelGearbox(gearbox)
            .withPlant(
                PlantCharacterization.ofInertia(
                        DCMotor.getKrakenX60(1), 1.0, KilogramSquareMeters.of(0.01))
                    .build())
            .withConstraints(constraints)
            .withVelocityOffsetStep(RadiansPerSecond.of(5))
            .withVoltageOffsetStep(Volts.of(1))
            .withVoltageCharacterization(voltageCharacterization)
            .withCurrentCharacterization(currentCharacterization)
            .build();

    assertSame(motor, constants.motor);
    assertSame(gearbox, constants.flywheelGearbox);
    assertEquals(
        PlantCharacterization.ofInertia(DCMotor.getKrakenX60(1), 1.0, KilogramSquareMeters.of(0.01))
            .build(),
        constants.plant);
    assertSame(constraints, constants.constraints);
    assertEquals(RadiansPerSecond.of(5), constants.velocityOffsetStep);
    assertEquals(Volts.of(1), constants.voltageOffsetStep);
    assertEquals(Amps.of(1.0), constants.currentOffsetStep);
    assertEquals(Value.of(0.05), constants.dutyCycleOffsetStep);
    assertSame(voltageCharacterization, constants.voltageCharacterization);
    assertSame(currentCharacterization, constants.currentCharacterization);

    GenericFlywheelConstants custom =
        constants.toBuilder().withDutyCycleOffsetStep(Value.of(0.1)).build();
    assertEquals(Value.of(0.1), custom.dutyCycleOffsetStep);
  }

  @Test
  public void shouldRequireEveryField() {
    assertThrows(NullPointerException.class, () -> GenericFlywheelConstants.builder().build());
  }
}
