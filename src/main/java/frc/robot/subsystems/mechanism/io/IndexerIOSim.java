package frc.robot.subsystems.mechanism.io;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexerIOSim implements IndexerIO {

    private final FlywheelSim m_rollerSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.002, 1.0),
        DCMotor.getKrakenX60Foc(1)
    );

    private double m_rollerVolts = 0.0;
    private double m_leftVolts   = 0.0;
    private double m_rightVolts  = 0.0;

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        m_rollerSim.update(0.02);

        inputs.rollerConnected    = true;
        inputs.rollerVelocityRPS  = m_rollerSim.getAngularVelocityRPM() / 60.0;
        inputs.rollerAppliedVolts = m_rollerVolts;
        inputs.rollerCurrentAmps  = m_rollerSim.getCurrentDrawAmps();

        inputs.leftConnected    = true;
        inputs.leftAppliedVolts = m_leftVolts;
        inputs.leftCurrentAmps  = Math.abs(m_leftVolts) * 1.0;

        inputs.rightConnected    = true;
        inputs.rightAppliedVolts = m_rightVolts;
        inputs.rightCurrentAmps  = Math.abs(m_rightVolts) * 1.0;
    }

    @Override
    public void setRollerVoltage(double volts) {
        m_rollerVolts = Math.max(-12.0, Math.min(12.0, volts));
        m_rollerSim.setInputVoltage(m_rollerVolts);
    }

    @Override
    public void setShaftVoltages(double leftVolts, double rightVolts) {
        m_leftVolts  = Math.max(-12.0, Math.min(12.0, leftVolts));
        m_rightVolts = Math.max(-12.0, Math.min(12.0, rightVolts));
    }

    @Override
    public void stopAll() {
        setRollerVoltage(0);
        setShaftVoltages(0, 0);
    }
}
