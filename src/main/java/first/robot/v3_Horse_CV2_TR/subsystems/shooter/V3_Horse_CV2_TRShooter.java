package first.robot.v3_Horse_CV2_TR.subsystems.shooter;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.mechanisms.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.robot.mechanisms.hood.GenericHood;
import first.robot.v3_Horse_CV2_TR.V3_Horse_CV2_TRRobotState;
import first.robot.v3_Horse_CV2_TR.V3_Horse_CV2_TRRobotState.FixedShotParameters;
import first.robot.v3_Horse_CV2_TR.V3_Horse_CV2_TRRobotState.FixedShots;
import first.robot.v3_Horse_CV2_TR.subsystems.shooter.V3_Horse_CV2_TRShooterConstants.ShooterGoal;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;
import org.wpilib.math.util.Units;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;

public class V3_Horse_CV2_TRShooter extends SubsystemBase {

  private final GenericFlywheel flywheel;
  private ShooterGoal shooterGoal;
  // Goal most recently pushed to the flywheel and hood in periodic
  private ShooterGoal appliedGoal;

  private final GenericHood hood;

  private final Trigger flywheelShootingTrigger;
  private final Trigger flywheelFeedingTrigger;

  private final Trigger hoodFeedingTrigger;

  private FixedShotParameters fixedShotParameters;

  public V3_Horse_CV2_TRShooter(MotorIOFactory flywheelMotors, MotorIOFactory hoodMotors) {

    Setpoint<AngularVelocityUnit> flywheelVelocitySetpoint =
        new Setpoint<AngularVelocityUnit>(
            RadiansPerSecond.of(0.0),
            V3_Horse_CV2_TRShooterConstants.SHOOT_CONSTANTS.velocityOffsetStep);
    Setpoint<VoltageUnit> flywheelVoltageSetpoint =
        new Setpoint<VoltageUnit>(
            Volts.of(0.0), V3_Horse_CV2_TRShooterConstants.SHOOT_CONSTANTS.voltageOffsetStep);

    Setpoint<AngleUnit> hoodAngleSetpoint =
        new Setpoint<AngleUnit>(
            Radians.of(0.0), V3_Horse_CV2_TRShooterConstants.HOOD_CONSTANTS.positionOffsetStep);
    Setpoint<VoltageUnit> hoodVoltageSetpoint =
        new Setpoint<VoltageUnit>(
            Volts.of(0.0), V3_Horse_CV2_TRShooterConstants.HOOD_CONSTANTS.voltageOffsetStep);

    this.flywheel =
        new GenericFlywheel(
            flywheelMotors,
            this,
            V3_Horse_CV2_TRShooterConstants.SHOOT_CONSTANTS,
            "Flywheel",
            flywheelVelocitySetpoint,
            flywheelVoltageSetpoint);
    this.hood =
        new GenericHood(
            hoodMotors,
            this,
            "Hood",
            V3_Horse_CV2_TRShooterConstants.HOOD_CONSTANTS,
            hoodAngleSetpoint,
            hoodVoltageSetpoint);
    this.shooterGoal = ShooterGoal.STOW;
    this.fixedShotParameters = FixedShots.FRONT_TOWER.getParameters();

    flywheelShootingTrigger =
        new Trigger(
            () ->
                Math.abs(
                        flywheel
                            .getVelocity()
                            .minus(flywheel.getVelocityGoal().getNewSetpoint())
                            .in(RadiansPerSecond))
                    <= V3_Horse_CV2_TRShooterConstants.SHOOT_CONSTANTS
                        .constraints
                        .goalTolerance()
                        .get()
                        .in(RadiansPerSecond));
    flywheelFeedingTrigger =
        new Trigger(
            () ->
                Math.abs(
                        flywheel
                            .getVelocity()
                            .minus(flywheel.getVelocityGoal().getNewSetpoint())
                            .in(RadiansPerSecond))
                    <= (V3_Horse_CV2_TRShooterConstants.SHOOT_CONSTANTS
                            .constraints
                            .goalTolerance()
                            .get()
                            .in(RadiansPerSecond)
                        + 35));

    hoodFeedingTrigger =
        new Trigger(
            () ->
                Math.abs(
                        hood.getPosition()
                            .minus(hood.getPositionGoal().getNewSetpoint())
                            .in(Radians))
                    <= (V3_Horse_CV2_TRShooterConstants.HOOD_CONSTANTS
                            .constraints
                            .goalTolerance()
                            .get()
                            .in(Radians)
                        + Units.degreesToRadians(5)));

    this.isHopperRunning = isHopperRunning;
  }

  @Trace
  public void periodic() {
    switch (shooterGoal) {
      case STOW:
        hood.setPositionGoal(
            V3_Horse_CV2_TRShooterConstants.HOOD_CONSTANTS.armParameters.minAngle());
        flywheel.stop();
        break;
      case SCORE:
        hood.setPositionGoal(V3_Horse_CV2_TRRobotState.getScoreAngle());
        flywheel.setVelocityGoal(
            V3_Horse_CV2_TRRobotState.getScoreVelocity(), getCurrentFeedforward());
        break;
      case FEED:
        hood.setPositionGoal(V3_Horse_CV2_TRRobotState.getFeedAngle());
        flywheel.setVelocityGoal(
            V3_Horse_CV2_TRRobotState.getFeedVelocity(), getCurrentFeedforward());
        break;
      case FIXED_SHOTS:
        hood.setPositionGoal(
            Rotation2d.fromDegrees(fixedShotParameters.hoodAngle().get().in(Degree)));
        flywheel.setVelocityGoal(
            fixedShotParameters.flywheelSpeed().get(), getCurrentFeedforward());
        break;
      case ZERO:
        hood.setPositionGoal(Radians.of(0.0));
        flywheel.stop();
        break;
      case STOP:
        hood.setPositionGoal(Radians.of(0.0));
        flywheel.stop();
        break;
      default:
        break;
    }

    hood.periodic();
    flywheel.periodic();
    appliedGoal = shooterGoal;

    Logger.recordOutput("Shooter/Goal", shooterGoal);

    Logger.recordOutput(
        "Shooter/Hood/Goal Degrees",
        String.format("%.1f", hood.getPositionGoal().getSetpoint().in(Degrees)));
    Logger.recordOutput(
        "Shooter/Hood/Offset Degrees",
        String.format("%.1f", hood.getPositionGoal().getOffset().in(Degrees)));
    Logger.recordOutput(
        "Shooter/Flywheel/Velocity Offset",
        flywheel.getVelocityGoal().getOffset().in(RadiansPerSecond));
    Logger.recordOutput(
        "Shooter/Flywheel/Velocity Magnitude",
        (int) Math.abs(flywheel.getVelocityGoal().getSetpoint().in(RadiansPerSecond)));

    Logger.recordOutput("Shooter/Flywheel At Goal", atGoal());

    Logger.recordOutput(
        "Shooter/Hood At Goal",
        (V3_Horse_CV2_TRRobotState.isInAllianceZone())
            ? hood.atPositionGoal()
            : hoodFeedingTrigger.getAsBoolean());
  }

  private Current getCurrentFeedforward() {
    return isHopperRunning.getAsBoolean()
        ? (flywheel
                .getFlywheelVelocity()
                .lt(flywheel.getVelocityGoal().getSetpoint().minus(RadiansPerSecond.of(5)))
            ? V3_Horse_CV2_TRShooterConstants.SHOOTER_FEEDFORWARD.get()
            : Amps.zero())
        : Amps.zero();
  }

  public Command setGoal(ShooterGoal shooterGoal) {
    return this.runOnce(() -> this.shooterGoal = shooterGoal);
  }

  public Command setGoal(Supplier<ShooterGoal> goalSupplier) {
    return this.run(() -> this.shooterGoal = goalSupplier.get());
  }

  public Command runFixedShot(FixedShots shot) {
    return Commands.runOnce(() -> fixedShotParameters = shot.getParameters())
        .andThen(setGoal(ShooterGoal.FIXED_SHOTS));
  }

  public boolean atGoal() {
    return shooterGoal == appliedGoal
        && (V3_Horse_CV2_TRRobotState.isInAllianceZone()
            ? (flywheelShootingTrigger.getAsBoolean() && hood.atPositionGoal())
            : (flywheelFeedingTrigger.getAsBoolean() && hoodFeedingTrigger.getAsBoolean()));
  }

  public Command waitUntilAtGoal() {
    return Commands.waitUntil(this::atGoal);
  }

  public Command waitUntilHoodAtGoal() {
    return hood.waitUntilAtPositionGoal();
  }

  public Command waitUntilFlywheelAtGoal() {
    return flywheel.waitUntilAtVelocityGoal();
  }

  public Command hoodSysId() {
    return hood.sysIdRoutine();
  }

  public Command flywheelSysId() {
    return flywheel.sysIdRoutineTorque();
  }

  public Command incrementFlywheelVelocity() {
    return Commands.runOnce(flywheel.getVelocityGoal()::increment);
  }

  public Command decrementFlywheelVelocity() {
    return Commands.runOnce(flywheel.getVelocityGoal()::decrement);
  }

  public Command incrementHoodAngle() {
    return Commands.runOnce(hood.getPositionGoal()::increment);
  }

  public Command decrementHoodAngle() {
    return Commands.runOnce(hood.getPositionGoal()::decrement);
  }

  public Command setHoodAngle(Angle angle) {
    return setGoal(ShooterGoal.IDLE).andThen(Commands.runOnce(() -> hood.setPositionGoal(angle)));
  }

  public void setHoodGains(Gains<?, AngleUnit, AngularVelocityUnit> hoodGains) {
    hood.updateGains(SlotConfig.position(hoodGains), GainSlot.ZERO);
  }

  public void setHoodConstraints(AngularPositionConstraints constraints) {
    hood.updateConstraints(constraints);
  }

  public void setFlywheelGains(
      Gains<VoltageUnit, AngularVelocityUnit, AngularVelocityUnit> flywheelGains) {
    flywheel.updateVoltageGains(flywheelGains);
  }

  public void setFlywheelConstraints(AngularVelocityConstraints constraints) {
    flywheel.updateConstraints(constraints);
  }

  public Command setFlywheelVelocity(AngularVelocity velocity) {
    return setGoal(ShooterGoal.IDLE)
        .andThen(Commands.runOnce(() -> flywheel.setVelocityGoal(velocity, Amps.of(0))));
  }

  public Command resetHoodZero() {
    return hood.home();
  }

  public Angle getHoodAngle() {
    return Radians.of(hood.getPosition().in(Radians));
  }

  public AngularVelocity getFlywheelVelocity() {
    return RadiansPerSecond.of(flywheel.getVelocity().in(RadiansPerSecond));
  }
}
