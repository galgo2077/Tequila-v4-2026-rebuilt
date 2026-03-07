package frc.robot.subsystems.mechanism.io;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

    @AutoLog
    class ClimberIOInputs {
        public boolean motorConnected    = false;
        public double  positionRevs      = 0.0;
        public double  velocityRPM       = 0.0;
        public double  appliedVolts      = 0.0;
        public double  supplyCurrentAmps = 0.0;  // Used by ClimberIOReal and ClimberIOSim
        public double  currentAmps       = 0.0;  // Alias used by climberSubsystem
        public double  tempCelsius       = 0.0;
        public boolean forwardLimit      = false;
        public boolean reverseLimit      = false;
    }

    /** Update sensor inputs — called every loop. */
    default void updateInputs(ClimberIOInputs inputs) {}

    /** Run climber at voltage (-12 to +12 V). */
    default void setVoltage(double volts) {}

    /** Stop motor output. */
    default void stop() {}

    /** Reset encoder position to zero. */
    default void resetClimberEncoder() {}

    /**
     * Enable or disable soft limits.
     * @param forwardEnable  enable forward (up) soft limit
     * @param reverseEnable  enable reverse (down) soft limit
     */
    default void enableClimberSoftLimits(boolean forwardEnable, boolean reverseEnable) {}
}