package frc.robot.subsystems.mechanism.io;

/**
 * IO interface for the Intake subsystem.
 * Two motors: roller (CAN ID 21) and extensor/rack-and-pinion (CAN ID 20).
 *
 * Homing support: the extensor must find its zero by driving against a hard stop.
 * The IO layer exposes resetExtensorEncoder() and enableExtensorSoftLimits() so
 * the subsystem can complete homing without calling TalonFX directly.
 */
public interface IntakeIO {

    public static class IntakeIOInputs {
        // ── Roller ────────────────────────────────────────────────────────────
        public boolean rollerConnected      = false;
        public double  rollerVelocityRPS    = 0.0;
        public double  rollerAppliedVolts   = 0.0;
        public double  rollerCurrentAmps    = 0.0;

        // ── Extensor ──────────────────────────────────────────────────────────
        public boolean extensorConnected    = false;
        /** Extensor position in motor rotations (0 = home/retracted). */
        public double  extensorPositionRevs = 0.0;
        public double  extensorAppliedVolts = 0.0;
        public double  extensorCurrentAmps  = 0.0;
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    // ── Roller ────────────────────────────────────────────────────────────────
    default void setRollerVoltage(double volts) {}

    // ── Extensor ──────────────────────────────────────────────────────────────
    default void setExtensorVoltage(double volts) {}

    /**
     * Zero the extensor encoder at the current (hard-stop) position.
     * Called by the subsystem at the end of the homing sequence.
     */
    default void resetExtensorEncoder() {}

    /**
     * Enable or disable extensor soft limits.
     * Call with (false, false) before homing, (true, true) after homing.
     */
    default void enableExtensorSoftLimits(boolean forward, boolean reverse) {}

    default void stopAll() {}
}
