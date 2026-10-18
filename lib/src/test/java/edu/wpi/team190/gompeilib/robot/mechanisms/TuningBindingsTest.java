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
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.VoltageUnit;

public class TuningBindingsTest extends GompeiLibTestBase {
  @Test
  public void testForwardsEachSlotsGainChanges() {
    Gains<VoltageUnit, AngleUnit, AngularVelocityUnit> gains0 =
        Gains.angularPosition(Volts)
            .withPrefix("bindings/slot0")
            .withKP(Volts.per(Rotations).ofNative(1.0))
            .build();
    Gains<VoltageUnit, AngularVelocityUnit, AngularVelocityUnit> gains1 =
        Gains.angularVelocity(Volts)
            .withPrefix("bindings/slot1")
            .withKP(Volts.per(RotationsPerSecond).ofNative(2.0))
            .build();
    SlotConfig slot0 = SlotConfig.position(gains0);
    SlotConfig slot1 = SlotConfig.velocity(gains1);
    MotorConfig config =
        MechanismTestSupport.config().toBuilder()
            .withSlot(GainSlot.ZERO, slot0)
            .withSlot(GainSlot.ONE, slot1)
            .build();
    MotorControllerGroup motors = mock(MotorControllerGroup.class);

    TuningBindings.bindGains(motors, config);
    MechanismTestSupport.fireChange(gains1);

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
