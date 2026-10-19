package edu.wpi.team190.gompeilib.io.motor.config;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import java.util.Optional;
import org.junit.jupiter.api.Test;

public class FollowerMotorTest {

  @Test
  public void testDefaultsToLeadersBusAndFactory() {
    FollowerMotor follower = new FollowerMotor(12, MotorAlignment.OPPOSED);

    assertEquals(12, follower.canId());
    assertEquals(MotorAlignment.OPPOSED, follower.alignment());
    assertEquals(Optional.empty(), follower.bus());
    assertEquals(Optional.empty(), follower.factory());
  }

  @Test
  public void testOnBus() {
    FollowerMotor follower =
        new FollowerMotor(12, MotorAlignment.ALIGNED).onBus(CANBusId.named("Other"));

    assertEquals(Optional.of(CANBusId.named("Other")), follower.bus());
    assertEquals(12, follower.canId());
    assertEquals(MotorAlignment.ALIGNED, follower.alignment());
  }

  @Test
  public void testBuiltBy() {
    FollowerMotor follower =
        new FollowerMotor(12, MotorAlignment.ALIGNED)
            .onBus(CANBusId.named("Other"))
            .builtBy(MotorIO.REPLAY_FACTORY);

    assertEquals(Optional.of(MotorIO.REPLAY_FACTORY), follower.factory());
    assertEquals(Optional.of(CANBusId.named("Other")), follower.bus());
  }

  @Test
  public void testFollowType() {
    FollowerMotor follower = new FollowerMotor(12, MotorAlignment.OPPOSED);
    assertEquals(FollowerMotor.FollowType.STANDARD, follower.followType());

    FollowerMotor diff =
        new FollowerMotor(13, MotorAlignment.ALIGNED, FollowerMotor.FollowType.DIFFERENTIAL);
    assertEquals(FollowerMotor.FollowType.DIFFERENTIAL, diff.followType());

    FollowerMotor strict = follower.withFollowType(FollowerMotor.FollowType.STRICT);
    assertEquals(FollowerMotor.FollowType.STRICT, strict.followType());
  }

  @Test
  public void testFourArgConstructor() {
    FollowerMotor follower =
        new FollowerMotor(
            15,
            MotorAlignment.ALIGNED,
            Optional.of(CANBusId.named("Rio")),
            Optional.of(MotorIO.REPLAY_FACTORY));
    assertEquals(15, follower.canId());
    assertEquals(MotorAlignment.ALIGNED, follower.alignment());
    assertEquals(FollowerMotor.FollowType.STANDARD, follower.followType());
    assertEquals(Optional.of(CANBusId.named("Rio")), follower.bus());
    assertEquals(Optional.of(MotorIO.REPLAY_FACTORY), follower.factory());
  }

  @Test
  public void shouldRejectNulls() {
    assertThrows(
        NullPointerException.class,
        () -> new FollowerMotor(1, null, Optional.empty(), Optional.empty()));
    assertThrows(
        NullPointerException.class,
        () ->
            new FollowerMotor(1, MotorAlignment.ALIGNED, null, Optional.empty(), Optional.empty()));
    assertThrows(
        NullPointerException.class,
        () -> new FollowerMotor(1, MotorAlignment.ALIGNED, null, Optional.empty()));
    assertThrows(
        NullPointerException.class,
        () -> new FollowerMotor(1, MotorAlignment.ALIGNED, Optional.empty(), null));
  }
}
