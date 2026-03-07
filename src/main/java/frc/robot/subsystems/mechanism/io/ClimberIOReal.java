package frc.robot.subsystems.mechanism.io;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CurrentLimits;

public class ClimberIOReal implements ClimberIO {

    private final TalonFX    m_motor      = new TalonFX(ClimberConstants.kClimberMotorId);
    private final VoltageOut m_voltageOut = new VoltageOut(0).withEnableFOC(true);

    private static final double kClimberMin = 0.0;
    private static final double kClimberMax = 50.0; // Measure on robot

    private final StatusSignal<edu.wpi.first.units.measure.Angle>   m_position;
    private final StatusSignal<edu.wpi.first.units.measure.Voltage>  m_appliedVolts;
    private final StatusSignal<edu.wpi.first.units.measure.Current>  m_supplyCurrent;

    public ClimberIOReal() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        // BRAKE is critical — holds the robot weight when the joystick is released
        cfg.MotorOutput.NeutralMode                          = NeutralModeValue.Brake;
        cfg.CurrentLimits.SupplyCurrentLimitEnable           = CurrentLimits.kNoLimitEnable; // No limit — needs full current to lift
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold    = kClimberMax;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold    = kClimberMin;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable       = false;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable       = false;
        m_motor.getConfigurator().apply(cfg);

        m_position      = m_motor.getPosition();
        m_appliedVolts  = m_motor.getMotorVoltage();
        m_supplyCurrent = m_motor.getSupplyCurrent();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.motorConnected    = m_motor.isConnected();
        inputs.positionRevs      = m_position.refresh().getValueAsDouble();
        inputs.appliedVolts      = m_appliedVolts.refresh().getValueAsDouble();
        inputs.supplyCurrentAmps = m_supplyCurrent.refresh().getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setControl(m_voltageOut.withOutput(volts));
    }

    @Override
    public void resetEncoder() {
        m_motor.setPosition(0.0);
    }

    @Override
    public void enableSoftLimits(boolean forward, boolean reverse) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode                       = NeutralModeValue.Brake;
        cfg.CurrentLimits.SupplyCurrentLimitEnable        = CurrentLimits.kNoLimitEnable;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kClimberMax;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kClimberMin;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = forward;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = reverse;
        m_motor.getConfigurator().apply(cfg);
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }
}
