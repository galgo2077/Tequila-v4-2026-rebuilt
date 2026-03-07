package frc.robot.subsystems.mechanism.io;

/**
 * IO interface for the Mobile (360°) Turret subsystem.
 * Three motors: rotation (ID 26), hood/angle (ID 27), flywheel (ID 28).
 *
 * Both the rotation and the hood require homing:
 *   - resetAngleEncoder() / enableAngleSoftLimits()  → hood homing
 *   - resetRotationEncoder() / enableRotationSoftLimits() → rotation homing
 */
public interface MobileTurretIO {

    public static class MobileTurretIOInputs {
        // ── Hood (angle) ──────────────────────────────────────────────────────
        public boolean angleConnected    = false;
        public double  anglePositionRevs = 0.0;
        public double  angleAppliedVolts = 0.0;
        public double  angleCurrentAmps  = 0.0;

        // ── Flywheel (shooter) ────────────────────────────────────────────────
        public boolean shooterConnected   = false;
        public double  shooterVelocityRPS = 0.0;
        public double  shooterAppliedVolts = 0.0;
        public double  shooterCurrentAmps  = 0.0;

        // ── Rotation ──────────────────────────────────────────────────────────
        public boolean rotationConnected    = false;
        public double  rotationPositionRevs = 0.0;
        public double  rotationAppliedVolts = 0.0;
        public double  rotationCurrentAmps  = 0.0;
    }

    default void updateInputs(MobileTurretIOInputs inputs) {}

    // ── Voltage setters ───────────────────────────────────────────────────────
    default void setAngleVoltage(double volts)    {}
    default void setShooterVoltage(double volts)  {}
    default void setRotationVoltage(double volts) {}

    // ── Homing support ────────────────────────────────────────────────────────
    default void resetAngleEncoder()    {}
    default void enableAngleSoftLimits(boolean forward, boolean reverse) {}

    default void resetRotationEncoder() {}
    default void enableRotationSoftLimits(boolean forward, boolean reverse) {}

    default void stopAll() {}
}
