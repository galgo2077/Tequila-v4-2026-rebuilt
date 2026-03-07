package frc.robot.subsystems.mechanism.io;

/**
 * IO interface for the Fixed Shooter subsystem.
 * Two motors: hood/angle (ID 29), flywheel (ID 30).
 * Hood requires homing against a hard stop.
 */
public interface FixedTurretIO {

    public static class FixedTurretIOInputs {
        // ── Hood ──────────────────────────────────────────────────────────────
        public boolean angleConnected    = false;
        public double  anglePositionRevs = 0.0;
        public double  angleAppliedVolts = 0.0;
        public double  angleCurrentAmps  = 0.0;

        // ── Flywheel ──────────────────────────────────────────────────────────
        public boolean shooterConnected    = false;
        public double  shooterVelocityRPS  = 0.0;
        public double  shooterAppliedVolts = 0.0;
        public double  shooterCurrentAmps  = 0.0;
    }

    default void updateInputs(FixedTurretIOInputs inputs) {}

    default void setAngleVoltage(double volts)   {}
    default void setShooterVoltage(double volts) {}

    default void resetAngleEncoder() {}
    default void enableAngleSoftLimits(boolean forward, boolean reverse) {}

    default void stopAll() {}
}
