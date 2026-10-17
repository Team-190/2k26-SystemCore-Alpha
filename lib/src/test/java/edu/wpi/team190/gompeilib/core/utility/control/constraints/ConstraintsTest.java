package edu.wpi.team190.gompeilib.core.utility.control.constraints;

import static org.junit.jupiter.api.Assertions.*;

import java.util.function.Consumer;
import org.junit.jupiter.api.Test;

public class ConstraintsTest {

  @Test
  public void testConstraintsInterface() {
    class DummyConstraints implements Constraints<DummyConstraints> {
      private Consumer<DummyConstraints> listener;

      @Override
      public void onChange(Consumer<DummyConstraints> listener) {
        this.listener = listener;
      }

      public void trigger() {
        if (listener != null) {
          listener.accept(this);
        }
      }
    }

    DummyConstraints dummy = new DummyConstraints();
    boolean[] called = new boolean[] {false};
    dummy.onChange(c -> called[0] = true);
    dummy.trigger();

    assertTrue(called[0]);
  }
}
