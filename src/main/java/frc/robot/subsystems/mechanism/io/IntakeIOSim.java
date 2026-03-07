package frc.robot.subsystems.mechanism.io;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulation implementation of IntakeIO.
 * Roller uses FlywheelSim; extensor uses a simple integrator (position += velocity * dt).
 */
public class IntakeIOSim implements IntakeIO {

    // Roller — treat as a flywheel, light inertia
    private final FlywheelSim m_rollerSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.002, 1.0),
        DCMotor.getKrakenX60Foc(1)
    );

    // Extensor — simple position integrator
    private double m_extensorPositionRevs = 0.0;
    private double m_extensorVoltage      = 0.0;
    private double m_rollerVoltage        = 0.0;

    private static final double kExtensorRevPerVoltPerSec = 1.5; // tune as needed

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        m_rollerSim.update(0.02);

        inputs.rollerConnected    = true;
        inputs.rollerVelocityRPS  = m_rollerSim.getAngularVelocityRPM() / 60.0;
        inputs.rollerAppliedVolts = m_rollerVoltage;
        inputs.rollerCurrentAmps  = m_rollerSim.getCurrentDrawAmps();

        // Simple extensor integration
        m_extensorPositionRevs += m_extensorVoltage * kExtensorRevPerVoltPerSec * 0.02;
        inputs.extensorConnected    = true;
        inputs.extensorPositionRevs = m_extensorPositionRevs;
        inputs.extensorAppliedVolts = m_extensorVoltage;
        inputs.extensorCurrentAmps  = Math.abs(m_extensorVoltage) * 1.5; // rough estimate
    }

    @Override
    public void setRollerVoltage(double volts) {
        m_rollerVoltage = Math.max(-12.0, Math.min(12.0, volts));
        m_rollerSim.setInputVoltage(m_rollerVoltage);
    }

    @Override
    public void setExtensorVoltage(double volts) {
        m_extensorVoltage = Math.max(-12.0, Math.min(12.0, volts));
    }

    @Override
    public void resetExtensorEncoder() {
        m_extensorPositionRevs = 0.0;
    }

    @Override
    public void enableExtensorSoftLimits(boolean forward, boolean reverse) {
        // No-op in simulation — subsystem logic handles limits via position checks
    }

    @Override
    public void stopAll() {
        setRollerVoltage(0);
        setExtensorVoltage(0);
    }
}
