package edu.wpi.team190.gompeilib.robot.mechanisms.roller;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.physics.PlantCharacterization;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.Units;

public class GenericRollerConstantsTest {
  @Test
  public void testConstantsAndBuilder() {
    MotorConfig motor =
        MotorConfig.builder()
            .withCanId(1)
            .withBus(CANBusId.named("rio"))
            .withCurrentLimits(
                CurrentLimits.fromDoubles()
                    .withSupplyCurrentLimit(30.0)
                    .withStatorCurrentLimit(30.0)
                    .build())
            .build();
    DCMotor gearbox = DCMotor.getNeo550(1);

    GenericRollerConstants constants =
        GenericRollerConstants.builder()
            .withMotor(motor)
            .withRollerGearbox(gearbox)
            .withPlant(
                PlantCharacterization.ofInertia(gearbox, 1.0, Units.KilogramSquareMeters.of(0.001))
                    .build())
            .withVoltageOffsetStep(Units.Volts.of(0.5))
            .build();

    assertSame(motor, constants.motor);
    assertSame(gearbox, constants.rollerGearbox);
    assertEquals(
        PlantCharacterization.ofInertia(gearbox, 1.0, Units.KilogramSquareMeters.of(0.001)).build(),
        constants.plant);
    assertEquals(Units.Volts.of(0.5), constants.voltageOffsetStep);
    assertEquals(Units.Amps.of(1.0), constants.currentOffsetStep);
    assertEquals(Units.Value.of(0.05), constants.dutyCycleOffsetStep);

    GenericRollerConstants custom =
        constants.toBuilder()
            .withCurrentOffsetStep(Units.Amps.of(2.5))
            .withDutyCycleOffsetStep(Units.Value.of(0.1))
            .build();
    assertEquals(Units.Amps.of(2.5), custom.currentOffsetStep);
    assertEquals(Units.Value.of(0.1), custom.dutyCycleOffsetStep);
  }

  @Test
  public void shouldRequireEveryField() {
    assertThrows(NullPointerException.class, () -> GenericRollerConstants.builder().build());
  }
}
