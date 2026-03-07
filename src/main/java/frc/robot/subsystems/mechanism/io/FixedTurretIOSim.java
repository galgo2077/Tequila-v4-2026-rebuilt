package frc.robot.subsystems.mechanism.io;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FixedTurretIOSim implements FixedTurretIO {

    private final FlywheelSim m_shooterSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.005, 1.0),
        DCMotor.getKrakenX60Foc(1)
    );

    private double m_angleRevs    = 0.0;
    private double m_angleVolts   = 0.0;
    private double m_shooterVolts = 0.0;

    private static final double kAngleRevPerVoltPerSec = 0.5;

    @Override
    public void updateInputs(FixedTurretIOInputs inputs) {
        m_shooterSim.update(0.02);
        m_angleRevs += m_angleVolts * kAngleRevPerVoltPerSec * 0.02;

        inputs.angleConnected    = true;
        inputs.anglePositionRevs = m_angleRevs;
        inputs.angleAppliedVolts = m_angleVolts;
        inputs.angleCurrentAmps  = Math.abs(m_angleVolts) * 1.0;

        inputs.shooterConnected    = true;
        inputs.shooterVelocityRPS  = m_shooterSim.getAngularVelocityRPM() / 60.0;
        inputs.shooterAppliedVolts = m_shooterVolts;
        inputs.shooterCurrentAmps  = m_shooterSim.getCurrentDrawAmps();
    }

    @Override public void setAngleVoltage(double v)   { m_angleVolts   = Math.max(-12, Math.min(12, v)); }
    @Override public void setShooterVoltage(double v) { m_shooterVolts = Math.max(-12, Math.min(12, v)); m_shooterSim.setInputVoltage(m_shooterVolts); }
    @Override public void resetAngleEncoder() { m_angleRevs = 0.0; }
    @Override public void enableAngleSoftLimits(boolean f, boolean r) {}
    @Override public void stopAll() { setAngleVoltage(0); setShooterVoltage(0); }
}
