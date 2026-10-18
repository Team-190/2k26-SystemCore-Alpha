package first.robot.v3_Horse_CV2_TR.subsystems.shooter;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Milliamps;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.FollowerMotor;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.MotorAlignment;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorDirection;
import edu.wpi.team190.gompeilib.io.motor.config.NeutralMode;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.io.motor.config.TorqueCurrentLimits;
import edu.wpi.team190.gompeilib.subsystems.flywheel.GenericFlywheelConstants;
import edu.wpi.team190.gompeilib.subsystems.hood.GenericHoodConstants;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;

public class V3_Horse_CV2_TRShooterConstants {

  public static final GenericFlywheelConstants SHOOT_CONSTANTS =
      GenericFlywheelConstants.builder()
          .withMotor(
              MotorConfig.builder()
                  .withCanId(41)
                  .withBus(CANBusId.port(CANPort.CAN_S4))
                  .withDirection(MotorDirection.CLOCKWISE_POSITIVE)
                  .withNeutralMode(NeutralMode.COAST)
                  .withCurrentLimits(
                      CurrentLimits.builder()
                          .withSupplyCurrentLimit(Amps.of(40.0))
                          .withStatorCurrentLimit(Amps.of(60.0))
                          .build())
                  // Never brake the flywheel with reverse torque current
                  .withTorqueCurrentLimits(
                      new TorqueCurrentLimits(TorqueCurrentLimits.NONE.peakForward(), Amps.of(0.0)))
                  .withGearRatio(30.0 / 18.0)
                  .withEnableFOC(true)
                  .withSlot(
                      GainSlot.ZERO,
                      new SlotConfig(
                          Gains.builder() // TODO: Use Real Value
                              .withPrefix("Shooter/Flywheel/Voltage")
                              .withKP(0.0)
                              .withKD(0.0)
                              .withKS(0.0)
                              .withKV(0.0)
                              .withKA(0.0)
                              .build()))
                  .withSlot(
                      GainSlot.ONE,
                      new SlotConfig(
                          Gains.builder() // TODO: Use Real Value
                              .withPrefix("Shooter/Flywheel/Torque")
                              .withKP(12)
                              .withKD(0)
                              .withKS(6.5)
                              .withKV(0.25)
                              .withKA(
                                  0.92854 * 2
                                      * Math.PI) // measured in A/(rad/s^2), Talon uses rotations
                              .build()))
                  .withFollower(new FollowerMotor(42, MotorAlignment.OPPOSED))
                  .withFollower(new FollowerMotor(43, MotorAlignment.ALIGNED))
                  .withFollower(new FollowerMotor(44, MotorAlignment.OPPOSED))
                  .build())
          .withFlywheelGearbox(DCMotor.getKrakenX60Foc(4))
          .withMomentOfInertia(KilogramSquareMeters.of(0.09473883059))
          .withConstraints(
              AngularVelocityConstraints.builder()
                  .withPrefix("Shooter/Flywheel")
                  .withMaxVelocity(RadiansPerSecond.of(1000))
                  .withMaxAcceleration(RadiansPerSecondPerSecond.of(1000))
                  .withGoalTolerance(RadiansPerSecond.of(10))
                  .build())
          .withVelocityOffsetStep(RadiansPerSecond.of(5))
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
                  .withGoalTolerance(Degrees.of(1))
                  .build())
          .withOffsetStep(Degrees.of(1))
          .withVoltageStep(Volts.of(0.5))
          .build();

  public enum ShooterGoal {
    SCORE,
    FEED,
    FIXED_SHOTS,
    STOW,
    ZERO,
    STOP,
    IDLE
  }
}
