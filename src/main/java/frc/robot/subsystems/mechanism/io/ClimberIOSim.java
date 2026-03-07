package frc.robot.subsystems.mechanism.io;

public class ClimberIOSim implements ClimberIO {

    private double m_appliedVolts = 0.0;
    private double m_positionRevs = 0.0;

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        // Simulate position based on applied voltage
        m_positionRevs += m_appliedVolts * 0.02 * (1.0 / 60.0);

        inputs.motorConnected    = true;
        inputs.positionRevs      = m_positionRevs;
        inputs.velocityRPM       = m_appliedVolts * 50.0;
        inputs.appliedVolts      = m_appliedVolts;
        inputs.supplyCurrentAmps = Math.abs(m_appliedVolts) * 4.0;
        inputs.currentAmps       = inputs.supplyCurrentAmps; // keep alias in sync
        inputs.tempCelsius       = 25.0;
        inputs.forwardLimit      = false;
        inputs.reverseLimit      = false;
    }

    @Override
    public void setVoltage(double volts) {
        m_appliedVolts = Math.max(-12.0, Math.min(12.0, volts));
    }

    @Override
    public void stop() {
        m_appliedVolts = 0.0;
    }

    @Override
    public void resetClimberEncoder() {
        m_positionRevs = 0.0;
    }

    @Override
    public void enableClimberSoftLimits(boolean forwardEnable, boolean reverseEnable) {
        // No-op in simulation — soft limits handled externally if needed
    }
}