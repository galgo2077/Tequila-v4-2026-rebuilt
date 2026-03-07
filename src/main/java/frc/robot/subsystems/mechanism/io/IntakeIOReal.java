package frc.robot.subsystems.mechanism.io;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.CurrentLimits;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final TalonFX m_rollerMotor   = new TalonFX(IntakeConstants.kIntakeMotorId);
    private final TalonFX m_extensorMotor = new TalonFX(IntakeConstants.kExtensorMotorId);

    private final VoltageOut m_voltageOut = new VoltageOut(0).withEnableFOC(true);

    // ── Extensor travel limits (motor revolutions) ────────────────────────────
    private static final double kExtensorRetracted = 0.0;
    private static final double kExtensorDeployed  = 15.0; // Measure on robot

    // ── Status signals ────────────────────────────────────────────────────────
    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_rollerVel;
    private final StatusSignal<edu.wpi.first.units.measure.Voltage>          m_rollerVolts;
    private final StatusSignal<edu.wpi.first.units.measure.Current>          m_rollerCurrent;

    private final StatusSignal<edu.wpi.first.units.measure.Angle>   m_extensorPos;
    private final StatusSignal<edu.wpi.first.units.measure.Voltage>  m_extensorVolts;
    private final StatusSignal<edu.wpi.first.units.measure.Current>  m_extensorCurrent;

    public IntakeIOReal() {
        // ── Roller config ──────────────────────────────────────────────────────
        TalonFXConfiguration rollerCfg = new TalonFXConfiguration();
        rollerCfg.MotorOutput.Inverted =
            IntakeConstants.kMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        rollerCfg.CurrentLimits.SupplyCurrentLimit       = CurrentLimits.kMechanismSupply;
        rollerCfg.CurrentLimits.SupplyCurrentLimitEnable = CurrentLimits.kMechanismLimitEnable;
        m_rollerMotor.getConfigurator().apply(rollerCfg);

        // ── Extensor config ────────────────────────────────────────────────────
        TalonFXConfiguration extCfg = new TalonFXConfiguration();
        extCfg.MotorOutput.NeutralMode                     = NeutralModeValue.Brake;
        extCfg.CurrentLimits.SupplyCurrentLimit            = CurrentLimits.kMechanismSupply;
        extCfg.CurrentLimits.SupplyCurrentLimitEnable      = CurrentLimits.kMechanismLimitEnable;
        // Soft limits OFF until homing is complete
        extCfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold  = kExtensorDeployed;
        extCfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold  = kExtensorRetracted;
        extCfg.SoftwareLimitSwitch.ForwardSoftLimitEnable     = false;
        extCfg.SoftwareLimitSwitch.ReverseSoftLimitEnable     = false;
        m_extensorMotor.getConfigurator().apply(extCfg);

        // ── Signals ────────────────────────────────────────────────────────────
        m_rollerVel     = m_rollerMotor.getVelocity();
        m_rollerVolts   = m_rollerMotor.getMotorVoltage();
        m_rollerCurrent = m_rollerMotor.getSupplyCurrent();

        m_extensorPos     = m_extensorMotor.getPosition();
        m_extensorVolts   = m_extensorMotor.getMotorVoltage();
        m_extensorCurrent = m_extensorMotor.getSupplyCurrent();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerConnected   = m_rollerMotor.isConnected();
        inputs.rollerVelocityRPS = m_rollerVel.refresh().getValueAsDouble();
        inputs.rollerAppliedVolts = m_rollerVolts.refresh().getValueAsDouble();
        inputs.rollerCurrentAmps  = m_rollerCurrent.refresh().getValueAsDouble();

        inputs.extensorConnected    = m_extensorMotor.isConnected();
        inputs.extensorPositionRevs = m_extensorPos.refresh().getValueAsDouble();
        inputs.extensorAppliedVolts = m_extensorVolts.refresh().getValueAsDouble();
        inputs.extensorCurrentAmps  = m_extensorCurrent.refresh().getValueAsDouble();
    }

    @Override
    public void setRollerVoltage(double volts) {
        m_rollerMotor.setControl(m_voltageOut.withOutput(volts));
    }

    @Override
    public void setExtensorVoltage(double volts) {
        m_extensorMotor.setControl(m_voltageOut.withOutput(volts));
    }

    @Override
    public void resetExtensorEncoder() {
        m_extensorMotor.setPosition(0.0);
    }

    @Override
    public void enableExtensorSoftLimits(boolean forward, boolean reverse) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode                          = NeutralModeValue.Brake;
        cfg.CurrentLimits.SupplyCurrentLimit                 = CurrentLimits.kMechanismSupply;
        cfg.CurrentLimits.SupplyCurrentLimitEnable           = CurrentLimits.kMechanismLimitEnable;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold    = kExtensorDeployed;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold    = kExtensorRetracted;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable       = forward;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable       = reverse;
        m_extensorMotor.getConfigurator().apply(cfg);
    }

    @Override
    public void stopAll() {
        m_rollerMotor.stopMotor();
        m_extensorMotor.stopMotor();
    }
}
