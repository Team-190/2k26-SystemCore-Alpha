package edu.wpi.team190.gompeilib.robot.mechanisms.hood;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.robot.mechanisms.CurrentHoming;
import edu.wpi.team190.gompeilib.robot.mechanisms.arm.Arm;
import edu.wpi.team190.gompeilib.robot.mechanisms.arm.ArmConstants;
import java.util.Optional;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.Subsystem;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.AngleUnit;

public class GenericHoodTest extends GompeiLibTestBase {
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

  private static ArmConstants constants(Optional<CurrentHoming.Config<AngleUnit>> homing) {
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
                .withLength(Meters.of(0.2))
                .withMomentOfInertia(KilogramSquareMeters.of(0.0001))
                .withSimulateGravity(false)
                .build())
        .withConstraints(AngularPositionConstraints.builder().build())
        .withVoltageOffsetStep(Volts.of(0.5))
        .withPositionOffsetStep(Degrees.of(1))
        .withHoming(homing)
        .build();
  }

  private GenericHood hood(ArmConstants constants) {
    return new GenericHood(
        factory,
        subsystem,
        "Hood",
        constants,
        new Setpoint<>(Degrees.of(0), Degrees.of(1)),
        new Setpoint<>(Volts.of(0), Volts.of(0.5)));
  }

  @Test
  public void testIsAnArmThatHomes() {
    CurrentHoming.Config<AngleUnit> homing =
        new CurrentHoming.Config<>(Volts.of(-1), Amps.of(40), Milliamps.of(500), Degrees.of(0));
    GenericHood hood = hood(constants(Optional.of(homing)));

    assertInstanceOf(Arm.class, hood);
    assertEquals("Shooter/Hood", hood.core().getTopic());
    assertSame(homing, hood.homing().getConfig());
    assertNotNull(hood.home());
  }

  @Test
  public void shouldRequireAHomingConfig() {
    assertThrows(IllegalArgumentException.class, () -> hood(constants(Optional.empty())));
  }
}
