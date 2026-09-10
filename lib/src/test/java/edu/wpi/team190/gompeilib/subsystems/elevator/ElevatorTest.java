package edu.wpi.team190.gompeilib.subsystems.elevator;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.subsystems.extension.Extension;
import edu.wpi.team190.gompeilib.subsystems.extension.ExtensionConstants;
import edu.wpi.team190.gompeilib.subsystems.extension.ExtensionIO;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Distance;

public class ElevatorTest {
  private ExtensionIO io;
  private Subsystem subsystem;
  private ExtensionConstants constants;

  @BeforeEach
  public void setUp() {
    org.wpilib.hardware.hal.HAL.initialize(500, 0);
    try {
      GompeiLib.deinit();
    } catch (Exception e) {
    }
    GompeiLib.init(RobotMode.SIM, false, 0.02);

    io = mock(ExtensionIO.class);
    subsystem = mock(Subsystem.class, CALLS_REAL_METHODS);
    when(subsystem.getName()).thenReturn("TestElevatorSubsystem");

    DCMotor motor = DCMotor.getNeo550(1);
    ExtensionConstants.ExtensionParameters params =
        ExtensionConstants.ExtensionParameters.builder()
            .withEXTENSION_MOTOR_CONFIG(motor)
            .withCARRIAGE_MASS_KG(15.0)
            .withMIN_LENGTH(Units.Meters.of(0.0))
            .withMAX_LENGTH(Units.Meters.of(1.5))
            .withNUM_MOTORS(1)
            .build();

    constants =
        ExtensionConstants.builder()
            .withLeaderCANID(5)
            .withExtensionGearRatio(10.0)
            .withDrumRadius(0.02)
            .withExtensionSupplyCurrentLimit(40.0)
            .withExtensionStatorCurrentLimit(40.0)
            .withExtensionParameters(params)
            .withSlot0Gains(Gains.fromDoubles().withPrefix("slot0").build())
            .withConstraints(
                LinearConstraints.fromMeasures()
                    .withPrefix("constraints")
                    .withMaxVelocity(Units.MetersPerSecond.of(2.0))
                    .withMaxAcceleration(Units.MetersPerSecondPerSecond.of(2.0))
                    .withGoalTolerance(Units.Meters.of(0.05))
                    .build())
            .withVerticalGravity(false)
            .withVoltageOffsetStep(Units.Volts.of(0.5))
            .withHeightOffsetStep(Units.Meters.of(0.05))
            .build();
  }

  @Test
  public void testElevator() {
    try (MockedStatic<Logger> mockLogger = mockStatic(Logger.class)) {
      Extension elevator = new Extension(constants, subsystem, 0, io);

      assertNotNull(elevator);

      // Test periodic in IDLE state
      elevator.periodic();
      verify(io).updateInputs(any());

      // Test open loop control state
      elevator.setVoltageGoal(Units.Volts.of(6.0));
      elevator.periodic();
      verify(io).setVoltageGoal(Units.Volts.of(6.0));

      // Test closed loop control state
      elevator.setPositionGoal(Units.Meters.of(1.0));
      elevator.periodic();
      verify(io).setPositionGoal(any(Distance.class));

      // Test setters/getters
      elevator.setVoltageGoal(
          new Setpoint<>(
              Units.Volts.of(2.0), Units.Volts.of(0.1), Units.Volts.of(-12), Units.Volts.of(12)));
      elevator.setPositionGoal(
          new Setpoint<>(
              Units.Meters.of(0.5),
              Units.Meters.of(0.01),
              Units.Meters.of(0.0),
              Units.Meters.of(1.5)));

      elevator.getExtensionPosition();

      // Test delegates
      elevator.setPosition(Units.Meters.of(0.4));
      verify(io).setPosition(Units.Meters.of(0.4));

      elevator.setGainSlot(GainSlot.ONE);
      verify(io).setGainSlot(GainSlot.ONE);

      elevator.updateGains(Gains.fromDoubles().withPrefix("slot0").build(), GainSlot.ZERO);
      verify(io).updateGains(any(), eq(GainSlot.ZERO));

      elevator.updateConstraints(
          LinearConstraints.fromMeasures()
              .withPrefix("constraints")
              .withGoalTolerance(Units.Meters.of(0.01))
              .withMaxVelocity(Units.MetersPerSecond.of(1.0))
              .withMaxAcceleration(Units.MetersPerSecondPerSecond.of(1.0))
              .build());
      verify(io).updateConstraints(any());

      // test goals
      when(io.atVoltageGoal(any())).thenReturn(true);
      assertTrue(elevator.atVoltageGoal());
      assertTrue(elevator.atVoltageGoal(Units.Volts.of(2.0)));

      when(io.atPositionGoal(any())).thenReturn(true);
      assertTrue(elevator.atPositionGoal());
      assertTrue(elevator.atPositionGoal(Units.Meters.of(0.5)));

      Command waitCmd = elevator.waitUntilAtGoal();
      assertNotNull(waitCmd);

      Command sysIdCmd = elevator.runSysIdRoutine();
      assertNotNull(sysIdCmd);
    }
  }
}
