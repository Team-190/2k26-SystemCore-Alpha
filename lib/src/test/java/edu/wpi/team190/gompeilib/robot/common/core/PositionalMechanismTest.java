package edu.wpi.team190.gompeilib.robot.common.core;

import static edu.wpi.team190.gompeilib.math.CustomUnits.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.robot.common.homing.CurrentHoming;
import java.util.Optional;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Subsystem;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularAccelerationUnit;
import org.wpilib.units.AngularVelocityUnit;

public class PositionalMechanismTest extends GompeiLibTestBase {
  /** The smallest positional mechanism: an angular one. */
  private static final class Pivot
      extends PositionalMechanism<
          AngleUnit, AngularVelocityUnit, AngularAccelerationUnit, AngularPositionConstraints> {
    Pivot(
        MotorIOFactory factory,
        Subsystem subsystem,
        AngularPositionConstraints constraints,
        Optional<CurrentHoming.Config<AngleUnit>> homing) {
      super(
          factory,
          MechanismTestSupport.config(),
          subsystem,
          "Pivot",
          MechanismConversion.angular(),
          constraints,
          () -> constraints.goalTolerance().get(),
          Radians.of(-1),
          Radians.of(1),
          constraints.characterizationMargin(),
          OutputType.CURRENT,
          new Setpoint<>(Radians.of(0), Radians.of(0.1)),
          new Setpoint<>(Volts.of(0), Volts.of(0.5)),
          Amps.of(1),
          Value.of(0.05),
          homing,
          CharacterizationConfig.voltage()
              .withRampRate(VoltsPerSecond.ofNative(1))
              .withStepOutput(Volts.of(4))
              .withQuasistaticTimeout(Seconds.of(1))
              .withDynamicTimeout(Seconds.of(1))
              .build());
    }

    @Override
    public MotionProfileConfig motionProfile(AngularPositionConstraints constraints) {
      return conversion()
          .trapezoidal(constraints.maxVelocity().get(), constraints.maxAcceleration().get());
    }

    @Override
    public void periodic() {
      core().periodic();
    }
  }

  private MockedStatic<Logger> logger;
  private MotorIOFactory factory;
  private Subsystem subsystem;
  private AngularPositionConstraints constraints;

  @BeforeEach
  public void setUp() {
    logger = mockStatic(Logger.class);
    factory = mock(MotorIOFactory.class);
    when(factory.family()).thenReturn("Mock");
    when(factory.create(any())).thenReturn(mock(MotorIO.class));
    subsystem = mock(Subsystem.class, CALLS_REAL_METHODS);
    when(subsystem.getName()).thenReturn("Test");
    constraints =
        AngularPositionConstraints.builder()
            .withMaxVelocity(RadiansPerSecond.of(1))
            .withMaxAcceleration(RadiansPerSecondPerSecond.of(2))
            .withGoalTolerance(Radians.of(0.05))
            .build();
  }

  @AfterEach
  public void tearDown() {
    logger.close();
  }

  @Test
  public void testWiresEveryControlToOneCore() {
    Pivot pivot = new Pivot(factory, subsystem, constraints, Optional.empty());
    MechanismCore core = pivot.core();
    assertEquals("Test/Pivot", core.getTopic());
    assertEquals(MechanismCore.IDLE, pivot.getMode());
    assertTrue(core.components().contains(pivot.positionControl()));
    assertTrue(core.components().contains(pivot.voltageControl()));
    assertTrue(core.components().contains(pivot.currentControl()));
    assertTrue(core.components().contains(pivot.dutyCycleControl()));
    assertEquals(OutputType.CURRENT, pivot.getPositionOutputType());
    assertSame(MechanismConversion.class, pivot.conversion().getClass());
  }

  @Test
  public void testConstrainsCharacterizationToItsTravel() {
    Pivot pivot = new Pivot(factory, subsystem, constraints, Optional.empty());
    var constraint = pivot.characterizationConstraint().orElseThrow();
    assertEquals(Radians.of(-1), constraint.lowerLimit());
    assertEquals(Radians.of(1), constraint.upperLimit());
    assertNotNull(pivot.characterize());
    assertNotNull(
        pivot.characterizationTarget(
            CharacterizationConfig.voltage()
                .withRampRate(VoltsPerSecond.ofNative(1))
                .withStepOutput(Volts.of(4))
                .withQuasistaticTimeout(Seconds.of(1))
                .withDynamicTimeout(Seconds.of(1))
                .build()));
  }

  @Test
  public void testHomesOnlyWhenConfigured() {
    Pivot unhomed = new Pivot(factory, subsystem, constraints, Optional.empty());
    assertThrows(IllegalStateException.class, unhomed::homing);

    Pivot homed =
        new Pivot(
            factory,
            subsystem,
            constraints,
            Optional.of(
                new CurrentHoming.Config<>(Volts.of(-1), Amps.of(-20), Amps.of(1), Radians.of(0))));
    assertTrue(homed.core().components().contains(homed.homing()));
  }
}
