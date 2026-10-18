package edu.wpi.team190.gompeilib.io.physics;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Method;
import org.junit.jupiter.api.Test;

public class MechanismSimTest {

  @Test
  public void testContractHasNoDefaults() {
    for (Method method : MechanismSim.class.getDeclaredMethods()) {
      assertFalse(method.isDefault(), method.getName() + " should be implemented by every model");
    }
  }

  @Test
  public void testEveryModelImplementsIt() {
    for (Class<?> model :
        new Class<?>[] {
          DCMotorMechanismSim.class,
          FlywheelMechanismSim.class,
          ElevatorMechanismSim.class,
          ArmMechanismSim.class
        }) {
      assertTrue(MechanismSim.class.isAssignableFrom(model), model.getSimpleName());
    }
  }
}
