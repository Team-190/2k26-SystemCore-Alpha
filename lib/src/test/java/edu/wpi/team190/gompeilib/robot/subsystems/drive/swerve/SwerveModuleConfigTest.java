package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;

public class SwerveModuleConfigTest extends GompeiLibTestBase {
  @Test
  public void testBuilderKeepsEveryField() {
    SwerveModuleConfig config = SwerveDriveConstantsTest.createModuleConfig(1, 2, 3, 0.5, -0.5);

    assertEquals(1, config.driveMotor().canId);
    assertEquals(2, config.turnMotor().canId);
    assertEquals(3, config.encoder().canId);
    assertEquals(new Translation2d(0.5, -0.5), config.location());
    assertEquals(0.0, config.drivePlant().kS());
    assertEquals(0.1, config.turnPlant().kS(), 1e-12);
  }

  @Test
  public void shouldRequireEveryField() {
    assertThrows(NullPointerException.class, () -> SwerveModuleConfig.builder().build());
  }
}
