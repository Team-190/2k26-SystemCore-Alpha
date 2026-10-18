package edu.wpi.team190.gompeilib.subsystems.arm;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;

public class ArmConstantsTest extends GompeiLibTestBase {
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
            .withMinAngle(Rotation2d.fromDegrees(-90))
            .withMaxAngle(Rotation2d.fromDegrees(90))
            .withLength(Meters.of(0.5))
            .withMomentOfInertia(KilogramSquareMeters.of(0.1))
            .withSimulateGravity(true)
            .build();
    AngularPositionConstraints constraints = AngularPositionConstraints.builder().build();

    ArmConstants constants =
        ArmConstants.builder()
            .withMotor(motor)
            .withArmParameters(parameters)
            .withConstraints(constraints)
            .withVoltageOffsetStep(Volts.of(0.5))
            .withPositionOffsetStep(Rotation2d.fromDegrees(5))
            .build();

    assertSame(motor, constants.motor);
    assertSame(parameters, constants.armParameters);
    assertSame(constraints, constants.constraints);
    assertEquals(Volts.of(0.5), constants.voltageOffsetStep);
    assertEquals(Rotation2d.fromDegrees(5), constants.positionOffsetStep);
    assertEquals(Meters.of(0.5), parameters.length());
    assertEquals(Rotation2d.fromDegrees(90), parameters.maxAngle());
    assertTrue(parameters.simulateGravity());
  }

  @Test
  public void shouldRequireEveryField() {
    assertThrows(NullPointerException.class, () -> ArmConstants.builder().build());
    assertThrows(NullPointerException.class, () -> ArmConstants.ArmParameters.builder().build());
  }
}
