package first.robot.v3_Horse_CV2_TR.subsystems.shooter;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Milliamps;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;
import edu.wpi.team190.gompeilib.subsystems.generic.hood.GenericHoodConstants;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;

public class V3_Horse_CV2_TRShooterConstants {

  public static final AngularVelocity TRENCH_SHOT_FLYWHEEL_SPEED =
      RadiansPerSecond.of(420.0); // TODO: Use Real Value
  public static final Rotation2d TRENCH_SHOT_HOOD_ANGLE =
      Rotation2d.fromDegrees(20.0); // TODO: Use Real Value

  public static final AngularVelocity HUB_SHOT_FLYWHEEL_SPEED =
      RadiansPerSecond.of(350); // TODO: Use Real Value
  public static final Rotation2d HUB_SHOT_HOOD_ANGLE =
      Rotation2d.fromDegrees(5.0); // TODO: Use Real Value

  public static final AngularVelocity TOWER_SHOT_FLYWHEEL_SPEED =
      RadiansPerSecond.of(402.00); // TODO: Use Real Value
  public static final Rotation2d TOWER_SHOT_HOOD_ANGLE =
      Rotation2d.fromDegrees(18.5); // TODO: Use Real Value

  public static final GenericFlywheelConstants SHOOT_CONSTANTS =
      GenericFlywheelConstants.builder()
          .withLeaderCANID(31) // TODO: Use Real Value
          .withLeaderInversion(InvertedValue.CounterClockwise_Positive)
          .withCanBus(CANBus.systemcore(0)) // TODO: Use Real Value
          .withEnableFOC(true)
          .withCurrentLimit(
              CurrentLimits.builder()
                  .withSupplyCurrentLimit(Amps.of(60.0)) // TODO: Use Real Value
                  .withStatorCurrentLimit(Amps.of(80.0)) // TODO: Use Real Value
                  .build())
          .withMomentOfInertia(0.09473883059) 
          .withGearRatio(18.0 / 30.0) 
          .withMotorConfig(DCMotor.getKrakenX60Foc(4))
          .withVoltageGains(
              Gains.builder() // TODO: Use Real Value
                  .withKP(new LoggedTunableNumber("Shooter/Flywheel/VoltageKp", 0.5))
                  .withKD(new LoggedTunableNumber("Shooter/Flywheel/VoltageKd", 0.0))
                  .withKS(new LoggedTunableNumber("Shooter/Flywheel/VoltageKs", 0.21467))
                  .withKV(new LoggedTunableNumber("Shooter/Flywheel/VoltageKv", 0.14015))
                  .withKA(new LoggedTunableNumber("Shooter/Flywheel/VoltageKa", 0.0045447))
                  .build())
          .withTorqueGains(
              Gains.builder() // TODO: Use Real Value
                  .withKP(new LoggedTunableNumber("Shooter/Flywheel/TorqueKp", 10))
                  .withKD(new LoggedTunableNumber("Shooter/Flywheel/TorqueKd", 0.1))
                  .withKS(new LoggedTunableNumber("Shooter/Flywheel/TorqueKs", 2.25))
                  .withKV(new LoggedTunableNumber("Shooter/Flywheel/TorqueKv", 0.067114))
                  .withKA(new LoggedTunableNumber("Shooter/Flywheel/TorqueKa", 0.11882))
                  .build())
          .withConstraints(
              AngularVelocityConstraints.builder()
                  .withMaxVelocity(
                      new LoggedTunableMeasure<>(
                          "Shooter/Flywheel/MaxVelocity", RadiansPerSecond.of(1000)))
                  .withMaxAcceleration(
                      new LoggedTunableMeasure<>(
                          "Shooter/Flywheel/MaxAcceleration", RadiansPerSecondPerSecond.of(1000)))
                  .withGoalTolerance(
                      new LoggedTunableMeasure<>(
                          "Shooter/Flywheel/GoalTolerance", RadiansPerSecond.of(5)))
                  .build())
          .withOpposedFollowerCANID(30) // TODO: Use Real Value
          .withVelocityOffsetStep(RadiansPerSecond.of(10)) // TODO: Use Real Value
          .withVoltageOffsetStep(Volts.of(1)) // TODO: Use Real Value
          .build();

  public static final GenericHoodConstants HOOD_CONSTANTS =
      GenericHoodConstants.builder()
          .withMotorCanId(32) // TODO: Use Real Value
          .withCanBus(CANBus.systemcore(0)) // TODO: Use Real Value
          .withGearRatio(70.0) // TODO: Use ASK CAM
          .withCurrentLimits(new CurrentLimits(40, 30)) // TODO: Use Real Value
          .withMomentOfInertia(0.0001)
          .withInvertedValue(InvertedValue.CounterClockwise_Positive)
          .withMotorConfig(DCMotor.getKrakenX44Foc(1))
          .withLengthMeters(0.211582)
          .withMinAngle(Rotation2d.fromDegrees(2))
          .withMaxAngle(Rotation2d.fromDegrees(40))
          .withZeroVoltage(Volts.of(1.0)) // TODO: Use Real Value
          .withZeroCurrentThreshold(Amps.of(40.0)) // TODO: Use Real Value
          .withZeroCurrentEpsilon(Milliamps.of(500)) // TODO: Use Real Value
          .withGains(
              Gains.builder() // TODO: Use Real Value
                  .withKP(new LoggedTunableNumber("Shooter/Hood/Kp", 600))
                  .withKD(new LoggedTunableNumber("Shooter/Hood/Kd", 2))
                  .withKS(new LoggedTunableNumber("Shooter/Hood/Ks", 0.32492))
                  .withKV(new LoggedTunableNumber("Shooter/Hood/Kv", 1.406))
                  .withKA(new LoggedTunableNumber("Shooter/Hood/Ka", 0))
                  .build())
          .withConstraints(
              AngularPositionConstraints.builder() // TODO: Use Real Value
                  .withMaxVelocity(
                      new LoggedTunableMeasure<>(
                          "Shooter/Hood/MaxVelocity", RadiansPerSecond.of(200)))
                  .withMaxAcceleration(
                      new LoggedTunableMeasure<>(
                          "Shooter/Hood/MaxAcceleration", RadiansPerSecondPerSecond.of(1000)))
                  .withGoalTolerance(
                      new LoggedTunableMeasure<>("Shooter/Hood/GoalTolerance", Degrees.of(1.0)))
                  .build())
          .withOffsetStep(Degrees.of(0.5)) // TODO: Use Real Value
          .withVoltageStep(Volts.of(0.5)) // TODO: Use Real Value
          .build();

  public enum ShooterGoal {
    SCORE,
    FEED,
    STOW,
    OVERRIDE_FLYWHEEL,
    OVERRIDE_HOOD,
    ZERO,
    STOP,
    IDLE
  }
}
