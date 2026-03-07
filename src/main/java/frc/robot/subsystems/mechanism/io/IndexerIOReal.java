package frc.robot.subsystems.mechanism.io;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.CurrentLimits;
import frc.robot.Constants.IndexerConstants;

public class IndexerIOReal implements IndexerIO {

    private final TalonFX m_rollerMotor = new TalonFX(IndexerConstants.kRollerMotorId);
    private final TalonFX m_leftMotor   = new TalonFX(IndexerConstants.kMotorLeftId);
    private final TalonFX m_rightMotor  = new TalonFX(IndexerConstants.kMotorRightId);

    private final VoltageOut m_voltageOut = new VoltageOut(0).withEnableFOC(true);

    // ── Signals ───────────────────────────────────────────────────────────────
    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_rollerVel;
    private final StatusSignal<edu.wpi.first.units.measure.Voltage>          m_rollerVolts;
    private final StatusSignal<edu.wpi.first.units.measure.Current>          m_rollerCurrent;

    private final StatusSignal<edu.wpi.first.units.measure.Voltage>  m_leftVolts;
    private final StatusSignal<edu.wpi.first.units.measure.Current>  m_leftCurrent;

    private final StatusSignal<edu.wpi.first.units.measure.Voltage>  m_rightVolts;
    private final StatusSignal<edu.wpi.first.units.measure.Current>  m_rightCurrent;

    public IndexerIOReal() {
        // ── Roller ────────────────────────────────────────────────────────────
        TalonFXConfiguration rollerCfg = new TalonFXConfiguration();
        rollerCfg.MotorOutput.NeutralMode                        = NeutralModeValue.Brake;
        rollerCfg.CurrentLimits.SupplyCurrentLimit               = CurrentLimits.kMechanismSupply;
        rollerCfg.CurrentLimits.SupplyCurrentLimitEnable         = CurrentLimits.kMechanismLimitEnable;
        m_rollerMotor.getConfigurator().apply(rollerCfg);

        // ── Left shaft ────────────────────────────────────────────────────────
        TalonFXConfiguration leftCfg = new TalonFXConfiguration();
        leftCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftCfg.MotorOutput.Inverted =
            IndexerConstants.kInvertLeft
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        leftCfg.CurrentLimits.SupplyCurrentLimit               = CurrentLimits.kMechanismSupply;
        leftCfg.CurrentLimits.SupplyCurrentLimitEnable         = CurrentLimits.kMechanismLimitEnable;
        m_leftMotor.getConfigurator().apply(leftCfg);

        // ── Right shaft ───────────────────────────────────────────────────────
        TalonFXConfiguration rightCfg = new TalonFXConfiguration();
        rightCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightCfg.MotorOutput.Inverted =
            IndexerConstants.kInvertRight
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        rightCfg.CurrentLimits.SupplyCurrentLimit               = CurrentLimits.kMechanismSupply;
        rightCfg.CurrentLimits.SupplyCurrentLimitEnable         = CurrentLimits.kMechanismLimitEnable;
        m_rightMotor.getConfigurator().apply(rightCfg);

        // ── Assign signals ────────────────────────────────────────────────────
        m_rollerVel     = m_rollerMotor.getVelocity();
        m_rollerVolts   = m_rollerMotor.getMotorVoltage();
        m_rollerCurrent = m_rollerMotor.getSupplyCurrent();

        m_leftVolts   = m_leftMotor.getMotorVoltage();
        m_leftCurrent = m_leftMotor.getSupplyCurrent();

        m_rightVolts   = m_rightMotor.getMotorVoltage();
        m_rightCurrent = m_rightMotor.getSupplyCurrent();
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.rollerConnected    = m_rollerMotor.isConnected();
        inputs.rollerVelocityRPS  = m_rollerVel.refresh().getValueAsDouble();
        inputs.rollerAppliedVolts = m_rollerVolts.refresh().getValueAsDouble();
        inputs.rollerCurrentAmps  = m_rollerCurrent.refresh().getValueAsDouble();

        inputs.leftConnected    = m_leftMotor.isConnected();
        inputs.leftAppliedVolts = m_leftVolts.refresh().getValueAsDouble();
        inputs.leftCurrentAmps  = m_leftCurrent.refresh().getValueAsDouble();

        inputs.rightConnected    = m_rightMotor.isConnected();
        inputs.rightAppliedVolts = m_rightVolts.refresh().getValueAsDouble();
        inputs.rightCurrentAmps  = m_rightCurrent.refresh().getValueAsDouble();
    }

    @Override
    public void setRollerVoltage(double volts) {
        m_rollerMotor.setControl(m_voltageOut.withOutput(volts));
    }

    @Override
    public void setShaftVoltages(double leftVolts, double rightVolts) {
        m_leftMotor.setControl(m_voltageOut.withOutput(leftVolts));
        m_rightMotor.setControl(m_voltageOut.withOutput(rightVolts));
    }

    @Override
    public void stopAll() {
        m_rollerMotor.stopMotor();
        m_leftMotor.stopMotor();
        m_rightMotor.stopMotor();
    }
}
