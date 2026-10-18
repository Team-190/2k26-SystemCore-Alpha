package edu.wpi.team190.gompeilib.io.phoenix;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDriveConstants;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDriveConstantsTest;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveModuleConfig;
import java.util.Queue;
import java.util.concurrent.locks.ReentrantLock;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;

@SuppressWarnings({"unchecked", "rawtypes"}) // Mockito mocks of generic Phoenix types
public class PhoenixOdometryThreadTest extends GompeiLibTestBase {
  @Test
  public void testPhoenixOdometryThread() {
    SwerveDriveConstants driveConstants = mock(SwerveDriveConstants.class);
    CANBusId canBus = CANBusId.named("rio");

    DCMotor motor = mock(DCMotor.class);
    SwerveModuleConfig moduleConst = SwerveDriveConstantsTest.createModuleConfig(1, 2, 3, 0.5, 0.5);
    OutputType outputType = OutputType.VOLTAGE;

    SwerveDriveConstants.DriveConfig driveConfig =
        new SwerveDriveConstants.DriveConfig(
            canBus,
            1, // pigeon2Id
            4.0, // maxLinearVelocityMetersPerSecond
            0.05, // wheelRadiusMeters
            motor,
            motor,
            moduleConst,
            moduleConst,
            moduleConst,
            moduleConst,
            outputType,
            outputType,
            0.8, // bumperWidth
            0.8, // bumperLength
            40.0, // robotMassKilograms
            0.6, // trackWidth
            2.0, // robotMOI
            40.0, // moduleCurrentLimit
            1.0 // wheelCOF
            );

    try {
      java.lang.reflect.Field configField =
          SwerveDriveConstants.class.getDeclaredField("driveConfig");
      configField.setAccessible(true);
      configField.set(driveConstants, driveConfig);

      java.lang.reflect.Field lockField =
          SwerveDriveConstants.class.getDeclaredField("reentrantLock");
      lockField.setAccessible(true);
      lockField.set(driveConstants, new ReentrantLock());

      java.lang.reflect.Field freqField =
          SwerveDriveConstants.class.getDeclaredField("odometryFrequency");
      freqField.setAccessible(true);
      freqField.set(driveConstants, 100.0);
    } catch (Exception e) {
      fail(e);
    }

    PhoenixOdometryThread thread = PhoenixOdometryThread.getInstance(driveConstants);
    assertNotNull(thread);

    // Test register signal
    StatusSignal<Angle> mockSignal = mock(StatusSignal.class);
    Queue<Double> signalQueue = thread.registerSignal(mockSignal);
    assertNotNull(signalQueue);

    // Test register generic signal
    Queue<Double> genericQueue = thread.registerSignal(() -> 42.0);
    assertNotNull(genericQueue);

    // Test makeTimestampQueue
    Queue<Double> tsQueue = thread.makeTimestampQueue();
    assertNotNull(tsQueue);
  }
}
