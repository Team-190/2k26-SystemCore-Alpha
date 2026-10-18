package edu.wpi.team190.gompeilib.subsystems.roller;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
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
            .withMomentOfInertia(Units.KilogramSquareMeters.of(0.001))
            .withVoltageOffsetStep(Units.Volts.of(0.5))
            .build();

    assertSame(motor, constants.motor);
    assertSame(gearbox, constants.rollerGearbox);
    assertEquals(Units.KilogramSquareMeters.of(0.001), constants.momentOfInertia);
    assertEquals(Units.Volts.of(0.5), constants.voltageOffsetStep);
  }

  @Test
  public void shouldRequireEveryField() {
    assertThrows(NullPointerException.class, () -> GenericRollerConstants.builder().build());
  }
}
