package edu.wpi.team190.gompeilib.io.motor.phoenix;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.team190.gompeilib.GompeiLib;
import edu.wpi.team190.gompeilib.io.motor.config.MotorAlignment;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.FollowerRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixOdometryThread;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import edu.wpi.team190.gompeilib.logging.Trace;
import org.wpilib.system.RobotController;

/**
 * A TalonFX running in Phoenix6 simulation, its rotor driven by a {@link MechanismSim}. Phoenix6
 * runs the requests; the mechanism supplies the physics.
 *
 * <p>The first motor on a mechanism drives it with the voltage Phoenix6 applies. Every motor on the
 * mechanism, including followers, has its rotor set from the mechanism's state, oriented by how it
 * is mounted relative to the driver.
 */
public class MotorIOTalonFXSim extends MotorIOTalonFX {
  private final MechanismSim mechanism;
  private final TalonFXSimState simState;
  private final boolean driver;

  /**
   * Creates a simulated TalonFX.
   *
   * @param config the motor's config
   * @param mechanism the physics model it drives or reads
   */
  public MotorIOTalonFXSim(MotorConfig config, MechanismSim mechanism) {
    this(config, mechanism, null);
  }

  /**
   * Creates a simulated TalonFX whose position is also sampled by an odometry thread.
   *
   * @param config the motor's config
   * @param mechanism the physics model it drives or reads
   * @param odometryThread the thread that samples position between cycles, or null for none
   */
  public MotorIOTalonFXSim(
      MotorConfig config, MechanismSim mechanism, PhoenixOdometryThread odometryThread) {
    super(config, odometryThread);
    this.mechanism = mechanism;
    this.simState = talonFX.getSimState();
    this.driver = mechanism.claimDriver(this);
    if (!driver
        && mechanism.driver() instanceof MotorIOTalonFXSim leader
        && leader.config.direction != config.direction) {
      // A mirrored opposed follower: its direction is flipped in config because it is mounted
      // facing the other way.
      simState.Orientation = ChassisReference.Clockwise_Positive;
    }
  }

  /**
   * Returns a factory for simulated TalonFXs sharing one mechanism. They hardware-follow each other
   * on the same bus, through Phoenix6's simulated following.
   *
   * @param mechanism the physics model
   * @return the factory
   */
  public static MotorIOFactory factory(MechanismSim mechanism) {
    return factory(mechanism, null);
  }

  /**
   * Returns a factory for simulated TalonFXs sharing one mechanism, their positions sampled by an
   * odometry thread.
   *
   * @param mechanism the physics model
   * @param odometryThread the thread, or null for none
   * @return the factory
   */
  public static MotorIOFactory factory(
      MechanismSim mechanism, PhoenixOdometryThread odometryThread) {
    return new MotorIOFactory() {
      @Override
      public MotorIO create(MotorConfig config) {
        return new MotorIOTalonFXSim(config, mechanism, odometryThread);
      }

      @Override
      public String family() {
        return FAMILY;
      }

      @Override
      public boolean supportsHardwareFollowing() {
        return true;
      }
    };
  }

  /**
   * Returns whether this motor drives its mechanism, rather than reading it.
   *
   * @return true if this motor drives the mechanism
   */
  public boolean isDriver() {
    return driver;
  }

  @Override
  public void setControl(MotorRequest request) {
    if (request instanceof FollowerRequest.Follow follow) {
      simState.Orientation =
          follow.alignment() == MotorAlignment.OPPOSED
              ? ChassisReference.Clockwise_Positive
              : ChassisReference.CounterClockwise_Positive;
    }
    super.setControl(request);
  }

  @Override
  @Trace
  public void updateInputs(MotorTelemetry telemetry) {
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    if (driver) {
      mechanism.setInputVoltage(Volts.of(simState.getMotorVoltage()));
      mechanism.update(Seconds.of(GompeiLib.getLoopPeriod()));
    }
    simState.setRawRotorPosition(mechanism.position().times(config.gearRatio));
    simState.setRotorVelocity(mechanism.velocity().times(config.gearRatio));
    simState.setRotorAcceleration(mechanism.acceleration().times(config.gearRatio));
    super.updateInputs(telemetry);
  }
}
