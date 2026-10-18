package edu.wpi.team190.gompeilib.subsystems.drive.swerve.phoenix;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.encoder.phoenix.AbsoluteEncoderIOCANcoder;
import edu.wpi.team190.gompeilib.io.motor.phoenix.MotorIOTalonFX;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixOdometryThread;
import edu.wpi.team190.gompeilib.subsystems.drive.swerve.SwerveDriveConstantsTest;
import edu.wpi.team190.gompeilib.subsystems.drive.swerve.SwerveModuleConfig;
import edu.wpi.team190.gompeilib.subsystems.drive.swerve.SwerveModuleHardware;
import org.junit.jupiter.api.Test;
import org.mockito.MockedConstruction;

public class SwerveModuleHardwareTalonFXTest extends GompeiLibTestBase {
  @Test
  public void testBuildsTalonFXMotorsAndACANcoder() {
    SwerveModuleConfig config = SwerveDriveConstantsTest.createModuleConfig(1, 2, 3, 0.5, 0.5);
    PhoenixOdometryThread thread = mock(PhoenixOdometryThread.class);
    try (MockedConstruction<AbsoluteEncoderIOCANcoder> encoders =
            mockConstruction(AbsoluteEncoderIOCANcoder.class);
        MockedConstruction<MotorIOTalonFX> motors = mockConstruction(MotorIOTalonFX.class)) {
      SwerveModuleHardware hardware = SwerveModuleHardwareTalonFX.create(config, thread);

      assertSame(encoders.constructed().get(0), hardware.encoder());
      assertEquals(MotorIOTalonFX.FAMILY, hardware.driveMotor().family());
      assertEquals(MotorIOTalonFX.FAMILY, hardware.turnMotor().family());

      hardware.driveMotor().create(config.driveMotor());
      hardware.turnMotor().create(config.turnMotor());
      assertEquals(2, motors.constructed().size());
    }
  }
}
