package edu.wpi.team190.gompeilib.io.physics;

import static org.wpilib.units.Units.KilogramSquareMeters;

import java.util.Objects;
import org.wpilib.units.measure.MomentOfInertia;

/**
 * One rotating part of a mechanism, such as a drum, a flywheel disk, a gear, or a motor's rotor,
 * and how fast it spins compared with the mechanism's output. A part spinning {@code r} times as
 * fast as the output stores {@code r²} times its kinetic energy there, so it adds {@code J·r²} to
 * the inertia the motors feel at the output.
 *
 * <p>List every part that turns, each at its own speed, and {@link PlantCharacterization#ofInertia}
 * sums them. A motor's rotor spins at the gear ratio, so on a high reduction it can outweigh
 * everything after the gearbox.
 *
 * @param momentOfInertia the part's moment of inertia about its own axis
 * @param speedRatio the part's speed divided by the output's speed
 */
public record Inertia(MomentOfInertia momentOfInertia, double speedRatio) {
  public Inertia {
    Objects.requireNonNull(momentOfInertia, "momentOfInertia");
    if (momentOfInertia.in(KilogramSquareMeters) < 0) {
      throw new IllegalArgumentException(
          "Moment of inertia must be non-negative, was " + momentOfInertia);
    }
    if (!Double.isFinite(speedRatio)) {
      throw new IllegalArgumentException("Speed ratio must be finite, was " + speedRatio);
    }
  }

  /**
   * A part that turns with the output, such as the flywheel's drum.
   *
   * @param momentOfInertia the part's moment of inertia
   * @return the part
   */
  public static Inertia atOutput(MomentOfInertia momentOfInertia) {
    return new Inertia(momentOfInertia, 1.0);
  }

  /**
   * A part that turns at its own speed, such as a disk geared up from the drum. Pass the motors'
   * rotors with the gear ratio, since they spin that much faster than the output.
   *
   * @param momentOfInertia the part's moment of inertia
   * @param speedRatio the part's speed divided by the output's speed
   * @return the part
   */
  public static Inertia geared(MomentOfInertia momentOfInertia, double speedRatio) {
    return new Inertia(momentOfInertia, speedRatio);
  }

  /**
   * Returns the inertia this part adds at the output.
   *
   * @return {@code J·r²}
   */
  public MomentOfInertia reflected() {
    return KilogramSquareMeters.of(
        momentOfInertia.in(KilogramSquareMeters) * speedRatio * speedRatio);
  }

  /**
   * Returns the inertia several parts add at the output together.
   *
   * @param parts the parts
   * @return the sum of each part's reflected inertia
   */
  public static MomentOfInertia total(Inertia... parts) {
    double total = 0.0;
    for (Inertia part : parts) {
      total += part.reflected().in(KilogramSquareMeters);
    }
    return KilogramSquareMeters.of(total);
  }
}
