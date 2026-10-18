package edu.wpi.team190.gompeilib.robot.mechanisms.flywheel;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;

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

    GenericFlywheelConstants constants =
        GenericFlywheelConstants.builder()
            .withMotor(motor)
            .withFlywheelGearbox(gearbox)
            .withMomentOfInertia(KilogramSquareMeters.of(0.01))
            .withConstraints(constraints)
            .withVelocityOffsetStep(RadiansPerSecond.of(5))
            .withVoltageOffsetStep(Volts.of(1))
            .build();

    assertSame(motor, constants.motor);
    assertSame(gearbox, constants.flywheelGearbox);
    assertEquals(KilogramSquareMeters.of(0.01), constants.momentOfInertia);
    assertSame(constraints, constants.constraints);
    assertEquals(RadiansPerSecond.of(5), constants.velocityOffsetStep);
    assertEquals(Volts.of(1), constants.voltageOffsetStep);
    assertEquals(Amps.of(1.0), constants.currentOffsetStep);
  }

  @Test
  public void shouldRequireEveryField() {
    assertThrows(NullPointerException.class, () -> GenericFlywheelConstants.builder().build());
  }
}
