package edu.wpi.team190.gompeilib.core.io.base;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class ConnectedIOTest {

  @Test
  public void testDefaultConnectedIsFalse() {
    ConnectedIO io = new ConnectedIO() {};
    assertFalse(io.isConnected());
  }
}
