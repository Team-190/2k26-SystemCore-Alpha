package edu.wpi.team190.gompeilib.io.motor.config;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class FeedbackSensorTest {

  @Test
  public void testInternalIsTheDefault() {
    assertInstanceOf(FeedbackSensor.Internal.class, FeedbackSensor.INTERNAL);
  }

  @Test
  public void testRemoteEncoder() {
    FeedbackSensor.RemoteEncoder remote =
        new FeedbackSensor.RemoteEncoder(3, RemoteSensorMode.SYNC);

    assertEquals(3, remote.canId());
    assertEquals(RemoteSensorMode.SYNC, remote.mode());
  }

  @Test
  public void shouldRequireAMode() {
    assertThrows(NullPointerException.class, () -> new FeedbackSensor.RemoteEncoder(3, null));
  }
}
