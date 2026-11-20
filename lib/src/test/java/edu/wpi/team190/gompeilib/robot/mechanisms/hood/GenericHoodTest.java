package edu.wpi.team190.gompeilib.robot.mechanisms.hood;

import static edu.wpi.team190.gompeilib.math.CustomUnits.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.physics.PlantCharacterization;
import edu.wpi.team190.gompeilib.robot.common.homing.CurrentHoming;
import edu.wpi.team190.gompeilib.robot.mechanisms.arm.ArmConstants;
import edu.wpi.team190.gompeilib.robot.mechanisms.arm.HomingArm;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.Subsystem;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.VoltageUnit;

public class GenericHoodTest extends GompeiLibTestBase {
  private static CharacterizationConfig<VoltageUnit> sysId() {
    return CharacterizationConfig.voltage()
        .withRampRate(VoltsPerSecond.ofNative(1))
        .withStepOutput(Volts.of(3))
        .withQuasistaticTimeout(Seconds.of(3))
        .withDynamicTimeout(Seconds.of(3))
        .build();
  }

  private MotorIOFactory factory;
  private Subsystem subsystem;

  @BeforeEach
  public void setUp() {
    factory = mock(MotorIOFactory.class);
    when(factory.family()).thenReturn("Mock");
    when(factory.create(any())).thenReturn(mock(MotorIO.class));
    subsystem = mock(Subsystem.class, CALLS_REAL_METHODS);
    when(subsystem.getName()).thenReturn("Shooter");
  }

  private static ArmConstants constants() {
    return ArmConstants.builder()
        .withMotor(
            MotorConfig.builder()
                .withCanId(45)
                .withBus(CANBusId.named("rio"))
                .withCurrentLimits(new CurrentLimits(40, 30))
                .build())
        .withArmParameters(
            ArmConstants.ArmParameters.builder()
                .withArmGearbox(DCMotor.getKrakenX44Foc(1))
                .withMinAngle(Degrees.of(0.5))
                .withMaxAngle(Degrees.of(35))
                .withPlant(
                    PlantCharacterization.ofInertia(
                            DCMotor.getKrakenX60(1), 50.0, KilogramSquareMeters.of(0.0001))
                        .build())
                .build())
        .withConstraints(AngularPositionConstraints.builder().build())
        .withVoltageOffsetStep(Volts.of(0.5))
        .withCharacterization(sysId())
        .withPositionOffsetStep(Degrees.of(1))
        .build();
  }

  private GenericHood hood(CurrentHoming.Config<AngleUnit> homing) {
    return new GenericHood(
        factory,
        subsystem,
        "Hood",
        constants(),
        homing,
        new Setpoint<>(Degrees.of(0), Degrees.of(1)),
        new Setpoint<>(Volts.of(0), Volts.of(0.5)));
  }

  @Test
  public void testIsAnArmThatHomes() {
    CurrentHoming.Config<AngleUnit> homing =
        new CurrentHoming.Config<>(
            CurrentHoming.Output.volts(Volts.of(-1)),
            Amps.of(40),
            Milliamps.of(500),
            Degrees.of(0));
    GenericHood hood = hood(homing);

    assertInstanceOf(HomingArm.class, hood);
    assertEquals("Shooter/Hood", hood.core().getTopic());
    assertSame(homing, hood.homing().getConfig());
    assertNotNull(hood.home());
  }
}
