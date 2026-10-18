package edu.wpi.team190.gompeilib.robot.mechanisms;

import edu.wpi.team190.gompeilib.io.motor.MotorControllerGroup;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.NeutralRequest;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Subsystem;

/**
 * The shared core of a mechanism: its motors, its log topic, and which control is driving it.
 *
 * <p>Components register with the core. Every loop, {@link #periodic()} reads the motors, has every
 * component log itself, and sends the active control's request. At most one control is active; the
 * core can instead be idle (sends nothing, so the last request stands) or stopped (sends neutral).
 * One-shot commands such as homing idle the core and drive the motors through {@link
 * #setControl(MotorRequest)}.
 */
public final class MechanismCore {
  /** The mode logged when no control is active and nothing is sent. */
  public static final String IDLE = "Idle";

  /** The mode logged while neutral is sent. */
  public static final String STOP = "Stop";

  /** A part of a mechanism that updates and logs itself every loop. */
  public interface Component {
    /**
     * Updates and logs this component, after the motors have been read.
     *
     * @param topic the mechanism's log topic
     */
    void periodic(String topic);
  }

  /** A component that can drive the motors while it is the active control. */
  public interface Control extends Component {
    /**
     * Returns the mode name logged while this control is active.
     *
     * @return the name
     */
    String name();

    /**
     * Returns the request to send this loop.
     *
     * @return the request
     */
    MotorRequest request();
  }

  @Getter private final MotorControllerGroup motors;
  @Getter private final String topic;
  private final List<Component> components = new ArrayList<>();

  private Control active;
  private boolean stopped;

  /**
   * Creates a core whose log topic is {@code <subsystem>/<name>}. It starts idle.
   *
   * @param motors the mechanism's motors
   * @param subsystem the subsystem that owns the mechanism
   * @param name the mechanism's name
   */
  public MechanismCore(MotorControllerGroup motors, Subsystem subsystem, String name) {
    this.motors = Objects.requireNonNull(motors, "motors");
    this.topic = subsystem.getName() + "/" + Objects.requireNonNull(name, "name");
  }

  /**
   * Registers a component so it is updated and logged every loop.
   *
   * @param component the component
   * @return the component
   */
  public <C extends Component> C add(C component) {
    components.add(Objects.requireNonNull(component, "component"));
    return component;
  }

  /**
   * Returns the registered components, in registration order.
   *
   * @return the components
   */
  public List<Component> components() {
    return Collections.unmodifiableList(components);
  }

  /**
   * Makes a control the one that drives the motors, from the next loop on.
   *
   * @param control the control
   * @throws IllegalArgumentException if the control isn't registered
   */
  public void activate(Control control) {
    if (!components.contains(control)) {
      throw new IllegalArgumentException(control.name() + " isn't registered with " + topic);
    }
    active = control;
    stopped = false;
  }

  /** Stops sending requests; the last request sent stays in effect. */
  public void idle() {
    active = null;
    stopped = false;
  }

  /** Sends neutral every loop until a control is activated. */
  public void stop() {
    active = null;
    stopped = true;
  }

  /**
   * Returns whether a control is the active one.
   *
   * @param control the control
   * @return true if it is active
   */
  public boolean isActive(Control control) {
    return active == control;
  }

  /**
   * Returns the active control's name, {@link #STOP}, or {@link #IDLE}.
   *
   * @return the mode
   */
  public String getMode() {
    if (active != null) {
      return active.name();
    }
    return stopped ? STOP : IDLE;
  }

  /**
   * Sends a request directly, without changing the mode. Idle the core first so the active control
   * doesn't replace it on the next loop.
   *
   * @param request the request
   */
  public void setControl(MotorRequest request) {
    motors.setControl(request);
  }

  /** Reads and logs the motors, updates every component, and sends the active request. */
  public void periodic() {
    motors.periodic(topic);
    Logger.recordOutput(topic + "/Mode", getMode());
    for (Component component : components) {
      component.periodic(topic);
    }
    if (active != null) {
      motors.setControl(active.request());
    } else if (stopped) {
      motors.setControl(new NeutralRequest.Neutral());
    }
  }
}
