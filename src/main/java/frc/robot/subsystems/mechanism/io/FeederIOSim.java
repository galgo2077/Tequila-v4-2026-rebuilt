package frc.robot.subsystems.mechanism.io;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulation implementation of FeederIO.
 * Uses WPILib FlywheelSim for physics-based kicker wheel behaviour.
 */
public class FeederIOSim implements FeederIO {

    // Small moment of inertia — kicker wheel is light
    private final FlywheelSim m_sim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60Foc(1), 0.001, 1.0),
        DCMotor.getKrakenX60Foc(1)
    );

    private double m_appliedVolts = 0.0;

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        m_sim.update(0.02);
        inputs.motorConnected    = true;
        inputs.velocityRPS       = m_sim.getAngularVelocityRPM() / 60.0;
        inputs.appliedVolts      = m_appliedVolts;
        inputs.supplyCurrentAmps = m_sim.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double volts) {
        m_appliedVolts = Math.max(-12.0, Math.min(12.0, volts));
        m_sim.setInputVoltage(m_appliedVolts);
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }
}
