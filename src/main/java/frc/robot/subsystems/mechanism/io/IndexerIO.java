package frc.robot.subsystems.mechanism.io;

/**
 * IO interface for the Indexer subsystem.
 * Three motors: roller (ID 22), left shaft (ID 23), right shaft (ID 24).
 *
 * The Y-valve routing logic (BOTH_SHOOTERS, TURRET_ONLY, etc.) lives in the
 * subsystem — the IO layer only exposes raw voltage setters.
 */
public interface IndexerIO {

    public static class IndexerIOInputs {
        // ── Roller ────────────────────────────────────────────────────────────
        public boolean rollerConnected   = false;
        public double  rollerVelocityRPS = 0.0;
        public double  rollerAppliedVolts = 0.0;
        public double  rollerCurrentAmps  = 0.0;

        // ── Left shaft ────────────────────────────────────────────────────────
        public boolean leftConnected   = false;
        public double  leftAppliedVolts = 0.0;
        public double  leftCurrentAmps  = 0.0;

        // ── Right shaft ───────────────────────────────────────────────────────
        public boolean rightConnected   = false;
        public double  rightAppliedVolts = 0.0;
        public double  rightCurrentAmps  = 0.0;
    }

    default void updateInputs(IndexerIOInputs inputs) {}

    default void setRollerVoltage(double volts) {}

    /** Set left and right mecanum shafts independently (Y-valve control). */
    default void setShaftVoltages(double leftVolts, double rightVolts) {}

    default void stopAll() {}
}
