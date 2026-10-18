package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.io.motor.MotorControllerGroup;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import org.junit.jupiter.api.Test;

public class TuningBindingsTest extends GompeiLibTestBase {
  @Test
  public void testForwardsEachSlotsGainChanges() {
    Gains slot0 = Gains.builder().withPrefix("bindings/slot0").withKP(1.0).build();
    Gains slot1 = Gains.builder().withPrefix("bindings/slot1").withKP(2.0).build();
    MotorConfig config =
        MechanismTestSupport.config().toBuilder()
            .withSlot(GainSlot.ZERO, new SlotConfig(slot0))
            .withSlot(GainSlot.ONE, new SlotConfig(slot1))
            .build();
    MotorControllerGroup motors = mock(MotorControllerGroup.class);

    TuningBindings.bindGains(motors, config);
    MechanismTestSupport.fireChange(slot1);

    verify(motors).updateGains(slot1, GainSlot.ONE);
    verify(motors, never()).updateGains(slot0, GainSlot.ZERO);
  }

  @Test
  public void testForwardsConstraintChangesThroughTheProfile() {
    LinearConstraints constraints =
        LinearConstraints.builder()
            .withPrefix("bindings/constraints")
            .withMaxVelocity(MetersPerSecond.of(1.0))
            .withMaxAcceleration(MetersPerSecondPerSecond.of(2.0))
            .build();
    MotorControllerGroup motors = mock(MotorControllerGroup.class);
    var drum = MechanismConversion.drum(Meters.of(0.5));

    TuningBindings.bindConstraints(
        motors,
        constraints,
        c -> drum.trapezoidal(c.maxVelocity().get(), c.maxAcceleration().get()));
    MechanismTestSupport.fireChange(constraints);

    verify(motors)
        .updateMotionProfile(
            MotionProfileConfig.trapezoidal(
                RadiansPerSecond.of(2.0), RadiansPerSecondPerSecond.of(4.0)));
  }
}
