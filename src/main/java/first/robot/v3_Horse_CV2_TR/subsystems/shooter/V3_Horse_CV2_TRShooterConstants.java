package first.robot.v3_Horse_CV2_TR.subsystems.shooter;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliamps;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

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
import edu.wpi.team190.gompeilib.io.motor.config.SoftLimits;
import edu.wpi.team190.gompeilib.io.motor.config.TorqueCurrentLimits;
import edu.wpi.team190.gompeilib.math.CustomUnits;
import edu.wpi.team190.gompeilib.robot.mechanisms.CurrentHoming;
import edu.wpi.team190.gompeilib.robot.mechanisms.arm.ArmConstants;
import edu.wpi.team190.gompeilib.robot.mechanisms.flywheel.GenericFlywheelConstants;
import edu.wpi.team190.gompeilib.sysid.SysIdConfig;
import edu.wpi.team190.gompeilib.tuning.GompeiTunables;
import java.util.Optional;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.system.DCMotor;
import org.wpilib.tunable.Tunable;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Current;

public class V3_Horse_CV2_TRShooterConstants {

  public static final Tunable<Current> SHOOTER_FEEDFORWARD =
      GompeiTunables.value("Shooter/Feedforward", Amps.of(40));

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
                  .withFollower(new FollowerMotor(42, MotorAlignment.OPPOSED))
                  .withFollower(new FollowerMotor(43, MotorAlignment.ALIGNED))
                  .withFollower(new FollowerMotor(44, MotorAlignment.OPPOSED))
                  .build())
          .withVoltageGains(
              Gains.angularVelocity(Volts) // TODO: Use Real Value
                  .withPrefix("Shooter/Flywheel/Voltage")
                  .build())
          .withCurrentGains(
              Gains.angularVelocity(Amps) // TODO: Use Real Value
                  .withPrefix("Shooter/Flywheel/Torque")
                  .withKP(Amps.per(RotationsPerSecond).ofNative(20))
                  .withKS(Amps.of(4))
                  .withKV(Amps.per(RotationsPerSecond).ofNative(0))
                  .withKA(Amps.per(RadiansPerSecondPerSecond).ofNative(0))
                  .build())
          .withFlywheelGearbox(DCMotor.getKrakenX60Foc(4))
          .withMomentOfInertia(KilogramSquareMeters.of(0.09473883059))
          .withConstraints(
              AngularVelocityConstraints.builder()
                  .withPrefix("Shooter/Flywheel")
                  .withMaxVelocity(RadiansPerSecond.of(1000))
                  .withMaxAcceleration(RadiansPerSecondPerSecond.of(1000))
                  .withGoalTolerance(RadiansPerSecond.of(1))
                  .build())
          .withVelocityOffsetStep(RadiansPerSecond.of(5))
          .withVoltageOffsetStep(Volts.of(1))
          .withVoltageSysId(
              SysIdConfig.voltage()
                  .withPrefix("Shooter/Flywheel/SysId/Voltage")
                  .withRampRate(CustomUnits.voltsPerSecond.ofNative(0.5))
                  .withStepOutput(Volts.of(8))
                  .withQuasistaticTimeout(Seconds.of(24))
                  .withDynamicTimeout(Seconds.of(10))
                  .withSettleTime(Seconds.of(6))
                  .build())
          .withCurrentSysId(
              SysIdConfig.current()
                  .withPrefix("Shooter/Flywheel/SysId/Torque")
                  .withRampRate(CustomUnits.ampsPerSecond.ofNative(5))
                  .withStepOutput(Amps.of(40))
                  .withQuasistaticTimeout(Seconds.of(10))
                  .withDynamicTimeout(Seconds.of(10))
                  .withSettleTime(Seconds.of(20))
                  .build())
          .build();

  private static final Angle HOOD_MIN_ANGLE = Degrees.of(0.5);
  private static final Angle HOOD_MAX_ANGLE = Degrees.of(35.0);

  public static final ArmConstants HOOD_CONSTANTS =
      ArmConstants.builder()
          .withMotor(
              MotorConfig.builder()
                  .withCanId(45)
                  .withBus(CANBusId.port(CANPort.CAN_S4))
                  .withDirection(MotorDirection.CLOCKWISE_POSITIVE)
                  .withCurrentLimits(new CurrentLimits(40, 30))
                  .withSoftLimits(SoftLimits.between(HOOD_MIN_ANGLE, HOOD_MAX_ANGLE))
                  .withGearRatio((56.0 / 12.0) * (150.0 / 10.0))
                  .withSlot(
                      GainSlot.ZERO,
                      SlotConfig.position(
                          Gains.angularPosition(Volts)
                              .withPrefix("Shooter/Hood")
                              .withKP(Volts.per(Rotations).ofNative(1152))
                              .withKD(Volts.per(RotationsPerSecond).ofNative(20))
                              .withKS(Volts.of(0.60085))
                              .withKV(Volts.per(RotationsPerSecond).ofNative(0.15846))
                              .withKA(Volts.per(RotationsPerSecondPerSecond).ofNative(0.13428))
                              .build()))
                  .build())
          .withArmParameters(
              ArmConstants.ArmParameters.builder()
                  .withArmGearbox(DCMotor.getKrakenX44Foc(1))
                  .withMinAngle(HOOD_MIN_ANGLE)
                  .withMaxAngle(HOOD_MAX_ANGLE)
                  .withLength(Meters.of(0.211582))
                  // Sim only. The effective inertia the characterized kA implies through the 70:1
                  // reduction; much lower and the tuned kP/kD go unstable in sim
                  .withMomentOfInertia(KilogramSquareMeters.of(0.5))
                  .withSimulateGravity(false)
                  .build())
          .withConstraints(
              AngularPositionConstraints.builder()
                  .withPrefix("Shooter/Hood")
                  .withMaxVelocity(RadiansPerSecond.of(37.5))
                  .withMaxAcceleration(RadiansPerSecondPerSecond.of(70))
                  .withGoalTolerance(Degrees.of(1))
                  .build())
          .withSysId(
              SysIdConfig.voltage()
                  .withPrefix("Shooter/Hood/SysId")
                  .withRampRate(CustomUnits.voltsPerSecond.ofNative(0.75))
                  .withStepOutput(Volts.of(4.5))
                  .withQuasistaticTimeout(Seconds.of(4))
                  .withDynamicTimeout(Seconds.of(4))
                  .withSettleTime(Seconds.of(1))
                  .build())
          .withPositionOffsetStep(Degrees.of(1))
          .withVoltageOffsetStep(Volts.of(0.5))
          .withHoming(
              Optional.of(
                  new CurrentHoming.Config<>(
                      Volts.of(-1.0), Amps.of(40.0), Milliamps.of(500), Degrees.of(0.0))))
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
