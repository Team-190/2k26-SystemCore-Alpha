package edu.wpi.team190.gompeilib.core.io.base;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

public class RequestControllableIOTest {

  @Test
  public void testSetControlHasNoDefault() throws NoSuchMethodException {
    assertFalse(RequestControllableIO.class.getMethod("setControl", Object.class).isDefault());
  }

  @Test
  public void testImplementationReceivesRequests() {
    List<String> received = new ArrayList<>();
    RequestControllableIO<String> io = received::add;

    io.setControl("first");
    io.setControl("second");

    assertEquals(List.of("first", "second"), received);
  }
}
