package frc.robot.subsystems.mechanism.io;

public class ClimberIOSim implements ClimberIO {

    private double m_positionRevs = 0.0;
    private double m_appliedVolts = 0.0;

    private static final double kRevPerVoltPerSec = 1.2; // tune as needed

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        m_positionRevs += m_appliedVolts * kRevPerVoltPerSec * 0.02;

        inputs.motorConnected    = true;
        inputs.positionRevs      = m_positionRevs;
        inputs.appliedVolts      = m_appliedVolts;
        inputs.supplyCurrentAmps = Math.abs(m_appliedVolts) * 4.0; // rough estimate for heavy lift
    }

    @Override
    public void setVoltage(double volts) {
        m_appliedVolts = Math.max(-12.0, Math.min(12.0, volts));
    }

    @Override
    public void resetEncoder() { m_positionRevs = 0.0; }

    @Override
    public void enableSoftLimits(boolean forward, boolean reverse) {}

    @Override
    public void stop() { setVoltage(0.0); }
}
