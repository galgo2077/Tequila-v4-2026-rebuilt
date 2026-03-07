package frc.robot.subsystems.mechanism.io;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.CurrentLimits;
import frc.robot.Constants.MobileTurretConstants;

public class MobileTurretIOReal implements MobileTurretIO {

    private final TalonFX m_angleMotor    = new TalonFX(MobileTurretConstants.kAngleMotorId);
    private final TalonFX m_shooterMotor  = new TalonFX(MobileTurretConstants.kShooterMotorId);
    private final TalonFX m_rotationMotor = new TalonFX(MobileTurretConstants.kRotationMotorId);

    private final VoltageOut m_voltageOut = new VoltageOut(0).withEnableFOC(true);

    // Travel limits (motor revolutions)
    private static final double kRotationMin = 0.0;
    private static final double kRotationMax = 40.0; // Measure on robot

    // ── Signals ───────────────────────────────────────────────────────────────
    private final StatusSignal<edu.wpi.first.units.measure.Angle>            m_anglePos;
    private final StatusSignal<edu.wpi.first.units.measure.Voltage>           m_angleVolts;
    private final StatusSignal<edu.wpi.first.units.measure.Current>           m_angleCurrent;

    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity>  m_shooterVel;
    private final StatusSignal<edu.wpi.first.units.measure.Voltage>           m_shooterVolts;
    private final StatusSignal<edu.wpi.first.units.measure.Current>           m_shooterCurrent;

    private final StatusSignal<edu.wpi.first.units.measure.Angle>            m_rotationPos;
    private final StatusSignal<edu.wpi.first.units.measure.Voltage>           m_rotationVolts;
    private final StatusSignal<edu.wpi.first.units.measure.Current>           m_rotationCurrent;

    public MobileTurretIOReal() {
        // ── Angle motor ───────────────────────────────────────────────────────
        TalonFXConfiguration angleCfg = new TalonFXConfiguration();
        angleCfg.MotorOutput.NeutralMode                         = NeutralModeValue.Brake;
        angleCfg.CurrentLimits.SupplyCurrentLimit                = CurrentLimits.kMechanismSupply;
        angleCfg.CurrentLimits.SupplyCurrentLimitEnable          = CurrentLimits.kMechanismLimitEnable;
        angleCfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold   = MobileTurretConstants.kHoodMaxRevs;
        angleCfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold   = MobileTurretConstants.kHoodMinRevs;
        angleCfg.SoftwareLimitSwitch.ForwardSoftLimitEnable      = false;
        angleCfg.SoftwareLimitSwitch.ReverseSoftLimitEnable      = false;
        m_angleMotor.getConfigurator().apply(angleCfg);

        // ── Rotation motor ────────────────────────────────────────────────────
        TalonFXConfiguration rotCfg = new TalonFXConfiguration();
        rotCfg.MotorOutput.NeutralMode                          = NeutralModeValue.Brake;
        rotCfg.CurrentLimits.SupplyCurrentLimit                 = CurrentLimits.kMechanismSupply;
        rotCfg.CurrentLimits.SupplyCurrentLimitEnable           = CurrentLimits.kMechanismLimitEnable;
        rotCfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold    = kRotationMax;
        rotCfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold    = kRotationMin;
        rotCfg.SoftwareLimitSwitch.ForwardSoftLimitEnable       = false;
        rotCfg.SoftwareLimitSwitch.ReverseSoftLimitEnable       = false;
        m_rotationMotor.getConfigurator().apply(rotCfg);

        // ── Shooter motor ─────────────────────────────────────────────────────
        TalonFXConfiguration shooterCfg = new TalonFXConfiguration();
        shooterCfg.MotorOutput.NeutralMode               = NeutralModeValue.Coast;
        shooterCfg.CurrentLimits.SupplyCurrentLimitEnable = CurrentLimits.kNoLimitEnable;
        m_shooterMotor.getConfigurator().apply(shooterCfg);

        // ── Assign signals ────────────────────────────────────────────────────
        m_anglePos     = m_angleMotor.getPosition();
        m_angleVolts   = m_angleMotor.getMotorVoltage();
        m_angleCurrent = m_angleMotor.getSupplyCurrent();

        m_shooterVel     = m_shooterMotor.getVelocity();
        m_shooterVolts   = m_shooterMotor.getMotorVoltage();
        m_shooterCurrent = m_shooterMotor.getSupplyCurrent();

        m_rotationPos     = m_rotationMotor.getPosition();
        m_rotationVolts   = m_rotationMotor.getMotorVoltage();
        m_rotationCurrent = m_rotationMotor.getSupplyCurrent();
    }

    @Override
    public void updateInputs(MobileTurretIOInputs inputs) {
        inputs.angleConnected    = m_angleMotor.isConnected();
        inputs.anglePositionRevs = m_anglePos.refresh().getValueAsDouble();
        inputs.angleAppliedVolts = m_angleVolts.refresh().getValueAsDouble();
        inputs.angleCurrentAmps  = m_angleCurrent.refresh().getValueAsDouble();

        inputs.shooterConnected    = m_shooterMotor.isConnected();
        inputs.shooterVelocityRPS  = m_shooterVel.refresh().getValueAsDouble();
        inputs.shooterAppliedVolts = m_shooterVolts.refresh().getValueAsDouble();
        inputs.shooterCurrentAmps  = m_shooterCurrent.refresh().getValueAsDouble();

        inputs.rotationConnected    = m_rotationMotor.isConnected();
        inputs.rotationPositionRevs = m_rotationPos.refresh().getValueAsDouble();
        inputs.rotationAppliedVolts = m_rotationVolts.refresh().getValueAsDouble();
        inputs.rotationCurrentAmps  = m_rotationCurrent.refresh().getValueAsDouble();
    }

    @Override public void setAngleVoltage(double volts)    { m_angleMotor.setControl(m_voltageOut.withOutput(volts)); }
    @Override public void setShooterVoltage(double volts)  { m_shooterMotor.setControl(m_voltageOut.withOutput(volts)); }
    @Override public void setRotationVoltage(double volts) { m_rotationMotor.setControl(m_voltageOut.withOutput(volts)); }

    @Override
    public void resetAngleEncoder() { m_angleMotor.setPosition(0.0); }

    @Override
    public void enableAngleSoftLimits(boolean forward, boolean reverse) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode                        = NeutralModeValue.Brake;
        cfg.CurrentLimits.SupplyCurrentLimit               = CurrentLimits.kMechanismSupply;
        cfg.CurrentLimits.SupplyCurrentLimitEnable         = CurrentLimits.kMechanismLimitEnable;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold  = MobileTurretConstants.kHoodMaxRevs;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold  = MobileTurretConstants.kHoodMinRevs;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable     = forward;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable     = reverse;
        m_angleMotor.getConfigurator().apply(cfg);
    }

    @Override
    public void resetRotationEncoder() { m_rotationMotor.setPosition(0.0); }

    @Override
    public void enableRotationSoftLimits(boolean forward, boolean reverse) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode                        = NeutralModeValue.Brake;
        cfg.CurrentLimits.SupplyCurrentLimit               = CurrentLimits.kMechanismSupply;
        cfg.CurrentLimits.SupplyCurrentLimitEnable         = CurrentLimits.kMechanismLimitEnable;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold  = kRotationMax;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold  = kRotationMin;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable     = forward;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable     = reverse;
        m_rotationMotor.getConfigurator().apply(cfg);
    }

    @Override
    public void stopAll() {
        m_angleMotor.stopMotor();
        m_shooterMotor.stopMotor();
        m_rotationMotor.stopMotor();
    }
}
