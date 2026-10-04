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
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelConstants;
import edu.wpi.team190.gompeilib.subsystems.generic.hood.GenericHoodConstants;
import org.wpilib.hardware.bus.CANPort;
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
          .withLeaderCANID(41)
          .withLeaderInversion(InvertedValue.Clockwise_Positive)
          .withCanBus(new CANBus(CANPort.CAN_S4))
          .withEnableFOC(true)
          .withCurrentLimit(
              CurrentLimits.builder()
                  .withSupplyCurrentLimit(Amps.of(60.0))
                  .withStatorCurrentLimit(Amps.of(80.0))
                  .build())
          .withMomentOfInertia(0.09473883059)
          .withGearRatio(30.0 / 18.0)
          .withMotorConfig(DCMotor.getKrakenX60Foc(4))
          .withVoltageGains(
              Gains.builder() // TODO: Use Real Value
                  .withPrefix("Shooter/Flywheel/Voltage")
                  .withKP(0.0)
                  .withKD(0.0)
                  .withKS(0.0)
                  .withKV(0.0)
                  .withKA(0.0)
                  .build())
          .withTorqueGains(
              Gains.builder() // TODO: Use Real Value
                  .withPrefix("Shooter/Flywheel/Torque")
                  .withKP(0)
                  .withKD(0)
                  .withKS(0.0)
                  .withKV(0.0)
                  .withKA(0.0)
                  .build())
          .withConstraints(
              AngularVelocityConstraints.builder()
                  .withPrefix("Shooter/Flywheel")
                  .withMaxVelocity(RadiansPerSecond.of(1000))
                  .withMaxAcceleration(RadiansPerSecondPerSecond.of(1000))
                  .withGoalTolerance(RadiansPerSecond.of(5))
                  .build())
          .withOpposedFollowerCANID(42)
          .withOpposedFollowerCANID(44)
          .withAlignedFollowerCANID(43)
          .withVelocityOffsetStep(RadiansPerSecond.of(10))
          .withVoltageOffsetStep(Volts.of(1))
          .build();

  public static final GenericHoodConstants HOOD_CONSTANTS =
      GenericHoodConstants.builder()
          .withMotorCanId(45)
          .withCanBus(new CANBus(CANPort.CAN_S4))
          .withGearRatio((56.0 / 12.0) * (150.0 / 10.0))
          .withCurrentLimits(new CurrentLimits(40, 30))
          .withMomentOfInertia(0.0001)
          .withInvertedValue(InvertedValue.Clockwise_Positive)
          .withMotorConfig(DCMotor.getKrakenX44Foc(1))
          .withLengthMeters(0.211582)
          .withMinAngle(Rotation2d.fromDegrees(0.5))
          .withMaxAngle(Rotation2d.fromDegrees(35.0))
          .withZeroVoltage(Volts.of(1.0))
          .withZeroCurrentThreshold(Amps.of(40.0))
          .withZeroCurrentEpsilon(Milliamps.of(500))
          .withGains(
              Gains.builder()
                  .withPrefix("Shooter/Hood")
                  .withKP(1152)
                  .withKD(20)
                  .withKS(0.60085)
                  .withKV(0.15846)
                  .withKA(0.13428)
                  .build())
          .withConstraints(
              AngularPositionConstraints.builder()
                  .withPrefix("Shooter/Hood")
                  .withMaxVelocity(RadiansPerSecond.of(37.5))
                  .withMaxAcceleration(RadiansPerSecondPerSecond.of(70))
                  .withGoalTolerance(Degrees.of(0.1))
                  .build())
          .withOffsetStep(Degrees.of(0.5))
          .withVoltageStep(Volts.of(0.5))
          .build();

  public enum ShooterGoal {
    SCORE,
    FEED,
    STOW,
    ZERO,
    STOP,
    IDLE
  }
}
