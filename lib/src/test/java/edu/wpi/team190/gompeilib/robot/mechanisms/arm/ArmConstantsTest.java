package edu.wpi.team190.gompeilib.robot.mechanisms.arm;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.math.CustomUnits;
import edu.wpi.team190.gompeilib.robot.mechanisms.CurrentHoming;
import edu.wpi.team190.gompeilib.sysid.SysIdConfig;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.VoltageUnit;

public class ArmConstantsTest extends GompeiLibTestBase {
  private static SysIdConfig<VoltageUnit> sysId() {
    return SysIdConfig.voltage()
        .withRampRate(CustomUnits.voltsPerSecond.ofNative(1))
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
            .withLength(Meters.of(0.5))
            .withMomentOfInertia(KilogramSquareMeters.of(0.1))
            .withSimulateGravity(true)
            .build();
    AngularPositionConstraints constraints = AngularPositionConstraints.builder().build();
    SysIdConfig<VoltageUnit> sysId = sysId();

    ArmConstants constants =
        ArmConstants.builder()
            .withMotor(motor)
            .withArmParameters(parameters)
            .withConstraints(constraints)
            .withVoltageOffsetStep(Volts.of(0.5))
            .withSysId(sysId)
            .withPositionOffsetStep(Degrees.of(5))
            .build();

    assertSame(motor, constants.motor);
    assertSame(parameters, constants.armParameters);
    assertSame(constraints, constants.constraints);
    assertEquals(Volts.of(0.5), constants.voltageOffsetStep);
    assertEquals(Degrees.of(5), constants.positionOffsetStep);
    assertSame(sysId, constants.sysId);
    assertEquals(Meters.of(0.5), parameters.length());
    assertEquals(Degrees.of(-90), parameters.minAngle());
    assertEquals(Degrees.of(90), parameters.maxAngle());
    assertEquals(KilogramSquareMeters.of(0.1), parameters.momentOfInertia());
    assertTrue(parameters.simulateGravity());
    assertTrue(constants.homing.isEmpty());
  }

  @Test
  public void testHoming() {
    CurrentHoming.Config<AngleUnit> homing =
        new CurrentHoming.Config<>(Volts.of(-1.0), Amps.of(40.0), Milliamps.of(500), Degrees.of(0));
    ArmConstants constants =
        ArmConstants.builder()
            .withMotor(
                MotorConfig.builder()
                    .withCanId(1)
                    .withBus(CANBusId.named("rio"))
                    .withCurrentLimits(new CurrentLimits(40, 40))
                    .build())
            .withArmParameters(
                ArmConstants.ArmParameters.builder()
                    .withArmGearbox(DCMotor.getKrakenX60(1))
                    .withMinAngle(Degrees.of(0))
                    .withMaxAngle(Degrees.of(30))
                    .withLength(Meters.of(0.2))
                    .withMomentOfInertia(KilogramSquareMeters.of(0.001))
                    .withSimulateGravity(false)
                    .build())
            .withConstraints(AngularPositionConstraints.builder().build())
            .withVoltageOffsetStep(Volts.of(0.5))
            .withSysId(sysId())
            .withPositionOffsetStep(Degrees.of(1))
            .withHoming(Optional.of(homing))
            .build();
    assertSame(homing, constants.homing.orElseThrow());
  }

  @Test
  public void shouldRequireEveryField() {
    assertThrows(NullPointerException.class, () -> ArmConstants.builder().build());
    assertThrows(NullPointerException.class, () -> ArmConstants.ArmParameters.builder().build());
  }
}
