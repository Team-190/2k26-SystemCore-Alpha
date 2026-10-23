package first.robot.v3_Horse_CV2_TR.subsystems.shooter;

import static edu.wpi.team190.gompeilib.math.CustomUnits.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig;
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
import edu.wpi.team190.gompeilib.io.physics.PlantCharacterization;
import edu.wpi.team190.gompeilib.robot.mechanisms.CurrentHoming;
import edu.wpi.team190.gompeilib.robot.mechanisms.arm.ArmConstants;
import edu.wpi.team190.gompeilib.robot.mechanisms.flywheel.GenericFlywheelConstants;
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
                  .withKP(AmpsPerRotationPerSecond.ofNative(10))
                  .withKS(Amps.of(3.6715))
                  .withKV(AmpsPerRadianPerSecond.ofNative(0.011621))
                  .withKA(AmpsPerRadianPerSecondSquared.ofNative(0.31749))
                  .build())
          .withFlywheelGearbox(DCMotor.getKrakenX60Foc(4))
          // From CAD until the torque characterization runs; then paste its printed plant here
          .withPlant(
              PlantCharacterization.ofInertia(
                      DCMotor.getKrakenX60Foc(4),
                      30.0 / 18.0,
                      KilogramSquareMeters.of(0.09473883059))
                  .build())
          .withConstraints(
              AngularVelocityConstraints.builder()
                  .withPrefix("Shooter/Flywheel")
                  .withMaxVelocity(RadiansPerSecond.of(1000))
                  .withMaxAcceleration(RadiansPerSecondPerSecond.of(1000))
                  .withGoalTolerance(RadiansPerSecond.of(1))
                  .build())
          .withVelocityOffsetStep(RadiansPerSecond.of(5))
          .withVoltageOffsetStep(Volts.of(1))
          .withVoltageCharacterization(
              CharacterizationConfig.voltage()
                  .withPrefix("Shooter/Flywheel/Characterization/Voltage")
                  .withRampRate(VoltsPerSecond.ofNative(0.5))
                  .withStepOutput(Volts.of(8))
                  .withQuasistaticTimeout(Seconds.of(24))
                  .withDynamicTimeout(Seconds.of(10))
                  .withSettleTime(Seconds.of(6))
                  .build())
          .withCurrentCharacterization(
              CharacterizationConfig.current()
                  .withPrefix("Shooter/Flywheel/Characterization/Torque")
                  .withRampRate(AmpsPerSecond.ofNative(5))
                  .withStepOutput(Amps.of(40))
                  .withQuasistaticTimeout(Seconds.of(10))
                  .withDynamicTimeout(Seconds.of(10))
                  .withSettleTime(Seconds.of(20))
                  .build())
          .build();

  private static final Angle HOOD_MIN_ANGLE = Degrees.of(0.5);
  private static final double HOOD_GEAR_RATIO = (56.0 / 12.0) * (150.0 / 10.0);
  private static final DCMotor HOOD_GEARBOX = DCMotor.getKrakenX44Foc(1);
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
                  .withGearRatio(HOOD_GEAR_RATIO)
                  .withSlot(
                      GainSlot.ZERO,
                      SlotConfig.position(
                          Gains.angularPosition(Volts)
                              .withPrefix("Shooter/Hood")
                              .withKP(VoltsPerRotation.ofNative(1152))
                              .withKD(VoltsPerRotationPerSecond.ofNative(20))
                              .withKS(Volts.of(0.60085))
                              .withKV(VoltsPerRotationPerSecond.ofNative(0.15846))
                              .withKA(VoltsPerRotationPerSecondSquared.ofNative(0.13428))
                              .build()))
                  .build())
          .withArmParameters(
              ArmConstants.ArmParameters.builder()
                  .withArmGearbox(HOOD_GEARBOX)
                  .withMinAngle(HOOD_MIN_ANGLE)
                  .withMaxAngle(HOOD_MAX_ANGLE)
                  // Measured kS and kA. kV is the motor model's: the hood's travel is too short for
                  // the quasistatic legs to reach a steady speed, so its measured kV is unreliable
                  .withPlant(
                      PlantCharacterization.voltage()
                          .withKS(Volts.of(0.60085))
                          .withKV(
                              VoltsPerRadianPerSecond.ofNative(HOOD_GEAR_RATIO / HOOD_GEARBOX.Kv))
                          .withKA(VoltsPerRotationPerSecondSquared.ofNative(0.13428))
                          .build())
                  .build())
          .withConstraints(
              AngularPositionConstraints.builder()
                  .withPrefix("Shooter/Hood")
                  .withMaxVelocity(RadiansPerSecond.of(37.5))
                  .withMaxAcceleration(RadiansPerSecondPerSecond.of(70))
                  .withGoalTolerance(Degrees.of(1))
                  .build())
          .withCharacterization(
              CharacterizationConfig.voltage()
                  .withPrefix("Shooter/Hood/Characterization")
                  .withRampRate(VoltsPerSecond.ofNative(0.75))
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
