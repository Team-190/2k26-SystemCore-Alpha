package first.robot.v3_Horse.subsystems.shooter;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import edu.wpi.team190.gompeilib.subsystems.generic.hood.GenericHood;
import edu.wpi.team190.gompeilib.subsystems.generic.hood.GenericHoodIO;
import first.robot.v3_Horse.V3_HorseRobotState;
import first.robot.v3_Horse.V3_HorseRobotState.FixedShotParameters;
import first.robot.v3_Horse.V3_HorseRobotState.FixedShots;
import first.robot.v3_Horse.subsystems.shooter.V3_HorseShooterConstants.ShooterGoal;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;

public class V3_HorseShooter extends SubsystemBase {

  private final GenericFlywheel flywheel;
  private ShooterGoal shooterGoal;
  // Goal most recently pushed to the flywheel and hood in periodic
  private ShooterGoal appliedGoal;

  private final GenericHood hood;

  private final Trigger flywheelShootingTrigger;
  private final Trigger flywheelFeedingTrigger;

  private final Trigger hoodFeedingTrigger;

  private FixedShotParameters fixedShotParameters;

  private BooleanSupplier isHopperRunning;

  public V3_HorseShooter(
      GenericFlywheelIO flywheelIO, GenericHoodIO hoodIO, BooleanSupplier isHopperRunning) {

    Setpoint<AngularVelocityUnit> flywheelVelocitySetpoint =
        new Setpoint<AngularVelocityUnit>(
            RadiansPerSecond.of(0.0), V3_HorseShooterConstants.SHOOT_CONSTANTS.velocityOffsetStep);
    Setpoint<VoltageUnit> flywheelVoltageSetpoint =
        new Setpoint<VoltageUnit>(
            Volts.of(0.0), V3_HorseShooterConstants.SHOOT_CONSTANTS.voltageOffsetStep);

    Setpoint<AngleUnit> hoodAngleSetpoint =
        new Setpoint<AngleUnit>(
            Radians.of(0.0), V3_HorseShooterConstants.HOOD_CONSTANTS.offsetStep);
    Setpoint<VoltageUnit> hoodVoltageSetpoint =
        new Setpoint<VoltageUnit>(
            Volts.of(0.0), V3_HorseShooterConstants.HOOD_CONSTANTS.voltageStep);

    this.flywheel =
        new GenericFlywheel(
            flywheelIO,
            this,
            V3_HorseShooterConstants.SHOOT_CONSTANTS,
            "",
            flywheelVelocitySetpoint,
            flywheelVoltageSetpoint);
    this.hood =
        new GenericHood(
            hoodIO,
            this,
            V3_HorseShooterConstants.HOOD_CONSTANTS,
            "",
            hoodAngleSetpoint,
            hoodVoltageSetpoint);
    this.shooterGoal = ShooterGoal.STOW;
    this.fixedShotParameters = FixedShots.FRONT_TOWER.getParameters();

    flywheelShootingTrigger =
        new Trigger(
            () ->
                Math.abs(
                        flywheel
                            .getFlywheelVelocity()
                            .minus(flywheel.getVelocityGoal().getNewSetpoint())
                            .in(RadiansPerSecond))
                    <= V3_HorseShooterConstants.SHOOT_CONSTANTS
                        .constraints
                        .goalTolerance()
                        .get()
                        .in(RadiansPerSecond));
    flywheelFeedingTrigger =
        new Trigger(
            () ->
                Math.abs(
                        flywheel
                            .getFlywheelVelocity()
                            .minus(flywheel.getVelocityGoal().getNewSetpoint())
                            .in(RadiansPerSecond))
                    <= (V3_HorseShooterConstants.SHOOT_CONSTANTS
                            .constraints
                            .goalTolerance()
                            .get()
                            .in(RadiansPerSecond)
                        + 35));

    hoodFeedingTrigger =
        new Trigger(
            () ->
                Math.abs(
                        hood.getAngle()
                            .getMeasure()
                            .minus(hood.getPositionGoal().getNewSetpoint())
                            .in(Radians))
                    <= (V3_HorseShooterConstants.HOOD_CONSTANTS
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
        hood.setPositionGoal(V3_HorseShooterConstants.HOOD_CONSTANTS.minAngle);
        flywheel.setVelocityGoal(getIdleVelocity(), getCurrentFeedforward());
        break;
      case SCORE:
        hood.setPositionGoal(V3_HorseRobotState.getScoreAngle());
        flywheel.setVelocityGoal(V3_HorseRobotState.getScoreVelocity(), getCurrentFeedforward());
        break;
      case FEED:
        hood.setPositionGoal(V3_HorseRobotState.getFeedAngle());
        flywheel.setVelocityGoal(V3_HorseRobotState.getFeedVelocity(), getCurrentFeedforward());
        break;
      case FIXED_SHOTS:
        hood.setPositionGoal(
            Rotation2d.fromDegrees(fixedShotParameters.hoodAngle().get().in(Degree)));
        flywheel.setVelocityGoal(
            fixedShotParameters.flywheelSpeed().get(), getCurrentFeedforward());
        break;
      case ZERO:
        hood.setPositionGoal(Rotation2d.ZERO);
        flywheel.stop();
        break;
      case STOP:
        hood.setPositionGoal(Rotation2d.ZERO);
        flywheel.setVelocityGoal(getIdleVelocity(), getCurrentFeedforward());
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
        (V3_HorseRobotState.isInAllianceZone())
            ? hood.atPositionGoal()
            : hoodFeedingTrigger.getAsBoolean());
  }

  private Current getCurrentFeedforward() {
    return isHopperRunning.getAsBoolean()
        ? (flywheel
                .getFlywheelVelocity()
                .lt(flywheel.getVelocityGoal().getSetpoint().minus(RadiansPerSecond.of(5)))
            ? V3_HorseShooterConstants.SHOOTER_FEEDFORWARD.get()
            : Amps.zero())
        : Amps.zero();
  }

  private AngularVelocity getIdleVelocity() {
    return V3_HorseRobotState.isInAllianceZone()
        ? V3_HorseRobotState.getScoreVelocity()
        : V3_HorseRobotState.FixedShots.FRONT_TOWER.getParameters().flywheelSpeed().get();
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
        && (V3_HorseRobotState.isInAllianceZone()
            ? (flywheelShootingTrigger.getAsBoolean() && hood.atPositionGoal())
            : (flywheelFeedingTrigger.getAsBoolean() && hoodFeedingTrigger.getAsBoolean()));
  }

  public Command waitUntilAtGoal() {
    return Commands.waitUntil(this::atGoal);
  }

  public Command waitUntilHoodAtGoal() {
    return hood.waitUntilAtGoal();
  }

  public Command waitUntilFlywheelAtGoal() {
    return flywheel.waitUntilAtGoal();
  }

  public Command hoodSysId() {
    return hood.runSysIdRoutine();
  }

  public Command flywheelSysId() {
    // IDLE keeps periodic() from overriding the routine's current requests with
    // stop()
    return setGoal(ShooterGoal.IDLE).andThen(flywheel.sysIdRoutineTorque());
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

  public Command setHoodAngle(Rotation2d angle) {
    return setGoal(ShooterGoal.IDLE).andThen(Commands.runOnce(() -> hood.setPositionGoal(angle)));
  }

  public void setHoodGains(Gains hoodGains) {
    hood.setGains(hoodGains);
  }

  public void setHoodConstraints(AngularPositionConstraints constraints) {
    hood.setProfile(constraints);
  }

  public void setFlywheelGains(Gains flywheelGains) {
    flywheel.updateGains(flywheelGains, GainSlot.ZERO);
  }

  public void setFlywheelConstraints(AngularVelocityConstraints constraints) {
    flywheel.updateConstraints(constraints);
  }

  public Command setFlywheelVelocity(AngularVelocity velocity) {
    return setGoal(ShooterGoal.IDLE)
        .andThen(Commands.runOnce(() -> flywheel.setVelocityGoal(velocity, Amps.of(0))));
  }

  public Command resetHoodZero() {
    return hood.resetHoodZero();
  }

  public Rotation2d getHoodAngle() {
    return hood.getAngle();
  }

  public AngularVelocity getFlywheelVelocity() {
    return flywheel.getFlywheelVelocity();
  }
}
