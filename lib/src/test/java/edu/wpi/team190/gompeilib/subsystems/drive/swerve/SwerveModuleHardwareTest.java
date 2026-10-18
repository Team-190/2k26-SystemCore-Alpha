package edu.wpi.team190.gompeilib.subsystems.drive.swerve;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.encoder.interfaces.AbsoluteEncoderIO;
import edu.wpi.team190.gompeilib.io.encoder.sim.AbsoluteEncoderIOSim;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.sim.MotorIOSim;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;

public class SwerveModuleHardwareTest extends GompeiLibTestBase {
  @Test
  public void testReplayReadsNothing() {
    assertSame(MotorIO.REPLAY_FACTORY, SwerveModuleHardware.REPLAY.driveMotor());
    assertSame(MotorIO.REPLAY_FACTORY, SwerveModuleHardware.REPLAY.turnMotor());
    assertNotNull(SwerveModuleHardware.REPLAY.encoder());
  }

  @Test
  public void testSimBuildsVendorNeutralSimulatedHardware() {
    SwerveModuleConfig config = SwerveDriveConstantsTest.createModuleConfig(1, 2, 3, 0.5, 0.5);
    SwerveModuleHardware hardware =
        SwerveModuleHardware.sim(config, DCMotor.getKrakenX60(1), DCMotor.getKrakenX44(1));

    assertInstanceOf(MotorIOSim.class, hardware.driveMotor().create(config.driveMotor()));
    assertInstanceOf(MotorIOSim.class, hardware.turnMotor().create(config.turnMotor()));
    assertInstanceOf(AbsoluteEncoderIOSim.class, hardware.encoder());
  }

  @Test
  public void testSimTakesTheMotorModelsFromTheDriveConfig() {
    SwerveModuleConfig config = SwerveDriveConstantsTest.createModuleConfig(1, 2, 3, 0.5, 0.5);
    SwerveDriveConstants.DriveConfig driveConfig = mock(SwerveDriveConstants.DriveConfig.class);
    when(driveConfig.driveModel()).thenReturn(DCMotor.getKrakenX60(1));
    when(driveConfig.turnModel()).thenReturn(DCMotor.getKrakenX44(1));

    SwerveModuleHardware hardware = SwerveModuleHardware.sim(config, driveConfig);

    assertInstanceOf(MotorIOSim.class, hardware.driveMotor().create(config.driveMotor()));
    assertInstanceOf(AbsoluteEncoderIOSim.class, hardware.encoder());
  }

  @Test
  public void shouldRequireEveryComponent() {
    MotorIOFactory factory = mock(MotorIOFactory.class);
    AbsoluteEncoderIO encoder = mock(AbsoluteEncoderIO.class);
    assertThrows(
        NullPointerException.class, () -> new SwerveModuleHardware(null, factory, encoder));
    assertThrows(
        NullPointerException.class, () -> new SwerveModuleHardware(factory, null, encoder));
    assertThrows(
        NullPointerException.class, () -> new SwerveModuleHardware(factory, factory, null));
  }
}
