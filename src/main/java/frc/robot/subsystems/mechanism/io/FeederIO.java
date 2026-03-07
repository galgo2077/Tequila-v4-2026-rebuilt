package frc.robot.subsystems.mechanism.io;

/**
 * IO interface for the Feeder subsystem (Kraken X44, CAN ID 25).
 *
 * Separates hardware from logic — the subsystem calls these methods without
 * knowing whether it is running on a real robot, in simulation, or in log replay.
 *
 * FUTURE: Add @AutoLog annotation to FeederIOInputs once AdvantageKit is
 * fully integrated (Phase 1 Step 1.1). That generates FeederIOInputsAutoLogged
 * which enables full log replay via Logger.processInputs().
 */
public interface FeederIO {

    // ─────────────────────────────────────────────────────────────────────────
    // INPUTS — all sensor data the subsystem needs, refreshed every 20 ms
    // ─────────────────────────────────────────────────────────────────────────
    public static class FeederIOInputs {
        /** True if the TalonFX is responding on the CAN bus. */
        public boolean motorConnected    = false;
        /** Rotor velocity in rotations per second. */
        public double  velocityRPS       = 0.0;
        /** Actual voltage applied to the motor (after any limiting). */
        public double  appliedVolts      = 0.0;
        /** Supply current drawn from the PDP/PDH in amps. */
        public double  supplyCurrentAmps = 0.0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // METHODS — all default (do nothing) so IOSim / IOReplay only override what matters
    // ─────────────────────────────────────────────────────────────────────────

    /** Populate inputs with latest hardware / simulation data. Called every loop cycle. */
    default void updateInputs(FeederIOInputs inputs) {}

    /** Command a voltage output (-12 to +12 V). */
    default void setVoltage(double volts) {}

    /** Coast to a stop (zero output). */
    default void stop() {}
}
