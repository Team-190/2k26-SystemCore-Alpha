package edu.wpi.team190.gompeilib.core.io.motor.request;

/**
 * A vendor-neutral motor control request. There is exactly one record per vendor control request,
 * so a vendor implementation is an exhaustive {@code switch} over this hierarchy.
 *
 * <p>All positions, velocities, and accelerations are at the gearbox output; mechanism-specific
 * conversion happens in the subsystem.
 */
public sealed interface MotorRequest
    permits AxisRequest, Differential, FollowerRequest, NeutralRequest {}
