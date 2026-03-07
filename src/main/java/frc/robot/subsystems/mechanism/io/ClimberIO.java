package frc.robot.subsystems.mechanism.io;

/**
 * IO interface for the Climber subsystem (Kraken X44, CAN ID 31).
 * Elevator with hard stop homing — must find zero before soft limits activate.
 */
public interface ClimberIO {

    public static class ClimberIOInputs {
        public boolean motorConnected    = false;
        /** Elevator position in motor rotations (0 = fully retracted / home). */
        public double  positionRevs      = 0.0;
        public double  appliedVolts      = 0.0;
        public double  supplyCurrentAmps = 0.0;
    }

    default void updateInputs(ClimberIOInputs inputs) {}

    default void setVoltage(double volts) {}

    default void resetEncoder() {}
    default void enableSoftLimits(boolean forward, boolean reverse) {}

    default void stop() {}
}
