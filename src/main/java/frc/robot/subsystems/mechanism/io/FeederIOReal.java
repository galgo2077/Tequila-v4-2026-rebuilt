package frc.robot.subsystems.mechanism.io;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.CurrentLimits;
import frc.robot.Constants.FeederConstants;

/**
 * Real hardware implementation of FeederIO.
 * All TalonFX calls live here — the subsystem never touches hardware directly.
 */
public class FeederIOReal implements FeederIO {

    private final TalonFX    m_motor      = new TalonFX(FeederConstants.kFeederMotorId);
    private final VoltageOut m_voltageOut = new VoltageOut(0).withEnableFOC(true);

    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_velocity;
    private final StatusSignal<edu.wpi.first.units.measure.Voltage>          m_appliedVolts;
    private final StatusSignal<edu.wpi.first.units.measure.Current>          m_supplyCurrent;

    public FeederIOReal() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        // Coast — don't abruptly stop a piece passing through the kicker
        cfg.MotorOutput.NeutralMode                        = NeutralModeValue.Coast;
        cfg.CurrentLimits.SupplyCurrentLimit               = CurrentLimits.kMechanismSupply;
        cfg.CurrentLimits.SupplyCurrentLimitEnable         = CurrentLimits.kMechanismLimitEnable;
        m_motor.getConfigurator().apply(cfg);

        m_velocity      = m_motor.getVelocity();
        m_appliedVolts  = m_motor.getMotorVoltage();
        m_supplyCurrent = m_motor.getSupplyCurrent();
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.motorConnected    = m_motor.isConnected();
        inputs.velocityRPS       = m_velocity.refresh().getValueAsDouble();
        inputs.appliedVolts      = m_appliedVolts.refresh().getValueAsDouble();
        inputs.supplyCurrentAmps = m_supplyCurrent.refresh().getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setControl(m_voltageOut.withOutput(volts));
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }
}
