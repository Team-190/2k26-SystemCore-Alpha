package edu.wpi.team190.gompeilib.io.motor.phoenix;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.team190.gompeilib.io.motor.config.MotorAlignment;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.FollowerRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixOdometryThread;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import org.wpilib.system.Notifier;
import org.wpilib.system.RobotController;

/**
 * A TalonFX running in Phoenix6 simulation, its rotor driven by a {@link MechanismSim}. Phoenix6
 * runs the requests; the mechanism supplies the physics.
 *
 * <p>The first motor on a mechanism drives it with the voltage Phoenix6 applies. Every motor on the
 * mechanism, including followers, has its rotor set from the mechanism's state, oriented by how it
 * is mounted relative to the driver.
 *
 * <p>Phoenix6 closes its loops on its own thread, so the physics runs on a shared 1 ms thread
 * rather than in the robot loop, matching the TalonFX's onboard loop. Otherwise gains tuned on
 * hardware see 20 ms-stale feedback and can go unstable in simulation.
 */
public class MotorIOTalonFXSim extends MotorIOTalonFX {
  /** How often the physics steps and is fed to Phoenix6. */
  static final double SIM_PERIOD_SECONDS = 0.001;

  private static final List<MotorIOTalonFXSim> SIMULATED = new CopyOnWriteArrayList<>();
  private static Notifier simThread;
  private static double lastSimTimeSeconds = Double.NaN;

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
    this(config, mechanism, odometryThread, true);
  }

  /**
   * Creates a simulated TalonFX, optionally without joining the sim thread so a test can step it
   * with {@link #simulate(double)}.
   *
   * @param config the motor's config
   * @param mechanism the physics model it drives or reads
   * @param odometryThread the thread that samples position between cycles, or null for none
   * @param threaded whether the shared sim thread steps this motor
   */
  MotorIOTalonFXSim(
      MotorConfig config,
      MechanismSim mechanism,
      PhoenixOdometryThread odometryThread,
      boolean threaded) {
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
    if (threaded) {
      register(this);
    }
  }

  private static synchronized void register(MotorIOTalonFXSim motor) {
    SIMULATED.add(motor);
    if (simThread == null) {
      simThread = new Notifier(MotorIOTalonFXSim::simulateAll);
      simThread.setName("TalonFXSim");
      simThread.startPeriodic(SIM_PERIOD_SECONDS);
    }
  }

  /** Stops the shared sim thread and forgets every motor on it, so tests don't leak it. */
  static synchronized void stopSimThread() {
    if (simThread != null) {
      simThread.close();
      simThread = null;
    }
    SIMULATED.clear();
    lastSimTimeSeconds = Double.NaN;
  }

  private static void simulateAll() {
    double now = RobotController.getTime() / 1e6;
    double dt = Double.isNaN(lastSimTimeSeconds) ? SIM_PERIOD_SECONDS : now - lastSimTimeSeconds;
    lastSimTimeSeconds = now;
    if (dt <= 0) {
      return;
    }
    for (MotorIOTalonFXSim motor : SIMULATED) {
      motor.simulate(dt);
    }
  }

  /**
   * Steps the mechanism, if this motor drives it, and feeds its state to Phoenix6.
   *
   * @param dtSeconds the time since the last step
   */
  void simulate(double dtSeconds) {
    synchronized (mechanism) {
      simState.setSupplyVoltage(RobotController.getBatteryVoltage());
      if (driver) {
        mechanism.setInputVoltage(Volts.of(simState.getMotorVoltage()));
        mechanism.update(Seconds.of(dtSeconds));
      }
      simState.setRawRotorPosition(mechanism.position().times(config.gearRatio));
      simState.setRotorVelocity(mechanism.velocity().times(config.gearRatio));
      simState.setRotorAcceleration(mechanism.acceleration().times(config.gearRatio));
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
}
