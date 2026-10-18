package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMechanism;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMotors;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.Command;

public class HomeableTest extends GompeiLibTestBase {
  @Test
  public void testHomeRunsTheHomingCommand() {
    TestMotors motors = new TestMotors();
    TestMechanism mechanism = new TestMechanism(motors);

    Command home = mechanism.home();
    home.initialize();
    home.execute();
    home.execute();
    verify(motors.motor, atLeastOnce())
        .setControl(new OpenLoopRequest.VoltageOutput(Volts.of(-1), true, CurrentHoming.OPTIONS));
    assertFalse(home.isFinished());
  }
}
