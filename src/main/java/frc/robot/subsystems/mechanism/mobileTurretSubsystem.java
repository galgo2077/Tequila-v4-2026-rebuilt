package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CurrentLimits;
import frc.robot.Constants.MobileTurretConstants;
import frc.robot.Constants.ShootingConfigs;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class mobileTurretSubsystem extends SubsystemBase {

    // ─────────────────────────────────────────────────────────────────────────
    // POSICIONES (en revoluciones del motor) — ajustar tras homing
    // ─────────────────────────────────────────────────────────────────────────
    // Rango del hood — definido en MobileTurretConstants como kHoodMinRevs / kHoodMaxRevs
    // kHoodMinDeg=15° → kHoodMinRevs≈0.1125  |  kHoodMaxDeg=60° → kHoodMaxRevs≈0.45
    private static final double kAngleHomingVolts            = -1.5;
    private static final double kAngleHomingCurrentThreshold =  8.0;  // Amps

    private static final double kRotationMin                    =  0.0;
    private static final double kRotationMax                    = 40.0; // Medir en robot
    private static final double kRotationHomingVolts            = -1.5;
    private static final double kRotationHomingCurrentThreshold = 10.0; // Amps

    // ─────────────────────────────────────────────────────────────────────────
    // MOTORES
    // ─────────────────────────────────────────────────────────────────────────
    private final TalonFX m_angleMotor    = new TalonFX(MobileTurretConstants.kAngleMotorId);
    private final TalonFX m_shooterMotor  = new TalonFX(MobileTurretConstants.kShooterMotorId);
    private final TalonFX m_rotationMotor = new TalonFX(MobileTurretConstants.kRotationMotorId);

    // ─────────────────────────────────────────────────────────────────────────
    // SEÑALES DE MONITOREO
    // ─────────────────────────────────────────────────────────────────────────
    private final StatusSignal<edu.wpi.first.units.measure.Angle>           m_anglePosSignal        = m_angleMotor.getPosition();
    private final StatusSignal<edu.wpi.first.units.measure.Current>         m_angleCurrentSignal    = m_angleMotor.getSupplyCurrent();
    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_shooterVelSignal      = m_shooterMotor.getVelocity();
    private final StatusSignal<edu.wpi.first.units.measure.Current>         m_shooterCurrentSignal  = m_shooterMotor.getSupplyCurrent();
    private final StatusSignal<edu.wpi.first.units.measure.Angle>           m_rotationPosSignal     = m_rotationMotor.getPosition();
    private final StatusSignal<edu.wpi.first.units.measure.Current>         m_rotationCurrentSignal = m_rotationMotor.getSupplyCurrent();

    // ─────────────────────────────────────────────────────────────────────────
    // ESTADO INTERNO — HOMING
    // ─────────────────────────────────────────────────────────────────────────
    private boolean m_angleHomed     = false;
    private boolean m_rotationHomed  = false;
    private boolean m_angleHoming    = false;
    private boolean m_rotationHoming = false;

    // ─────────────────────────────────────────────────────────────────────────
    // ESTADO INTERNO — VISIÓN
    // ─────────────────────────────────────────────────────────────────────────
    private double  m_cachedTx       = 0.0;
    private double  m_cachedDistance = 0.0;
    private boolean m_hasTarget      = false;

    // ─────────────────────────────────────────────────────────────────────────
    // CONSTRUCTOR
    // ─────────────────────────────────────────────────────────────────────────
    public mobileTurretSubsystem() {
        // Angle motor
        TalonFXConfiguration angleConfig = new TalonFXConfiguration();
        angleConfig.MotorOutput.NeutralMode                       = NeutralModeValue.Brake;
        angleConfig.CurrentLimits.SupplyCurrentLimit              = CurrentLimits.kMechanismSupply;
        angleConfig.CurrentLimits.SupplyCurrentLimitEnable        = CurrentLimits.kMechanismLimitEnable;
        angleConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MobileTurretConstants.kHoodMaxRevs;
        angleConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MobileTurretConstants.kHoodMinRevs;
        angleConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable    = false;
        angleConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable    = false;
        m_angleMotor.getConfigurator().apply(angleConfig);

        // Rotation motor
        TalonFXConfiguration rotationConfig = new TalonFXConfiguration();
        rotationConfig.MotorOutput.NeutralMode                       = NeutralModeValue.Brake;
        rotationConfig.CurrentLimits.SupplyCurrentLimit              = CurrentLimits.kMechanismSupply;
        rotationConfig.CurrentLimits.SupplyCurrentLimitEnable        = CurrentLimits.kMechanismLimitEnable;
        rotationConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kRotationMax;
        rotationConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kRotationMin;
        rotationConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable    = false;
        rotationConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable    = false;
        m_rotationMotor.getConfigurator().apply(rotationConfig);

        // Shooter motor
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.NeutralMode              = NeutralModeValue.Coast;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = CurrentLimits.kNoLimitEnable;
        m_shooterMotor.getConfigurator().apply(shooterConfig);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // MÉTODOS DE ACCIÓN
    // ─────────────────────────────────────────────────────────────────────────
    public void setAngleVoltage(double volts)    { m_angleMotor.setControl(new VoltageOut(volts)); }
    public void setShooterVoltage(double volts)  { m_shooterMotor.setControl(new VoltageOut(volts)); }
    public void setRotationVoltage(double volts) { m_rotationMotor.setControl(new VoltageOut(volts)); }

    public void stop() {
        m_angleMotor.stopMotor();
        m_shooterMotor.stopMotor();
        m_rotationMotor.stopMotor();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // HOMING — HELPERS PRIVADOS
    // ─────────────────────────────────────────────────────────────────────────
    private void completeAngleHoming() {
        m_angleMotor.setPosition(0.0);
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode                       = NeutralModeValue.Brake;
        cfg.CurrentLimits.SupplyCurrentLimit              = CurrentLimits.kMechanismSupply;
        cfg.CurrentLimits.SupplyCurrentLimitEnable        = CurrentLimits.kMechanismLimitEnable;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MobileTurretConstants.kHoodMaxRevs;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MobileTurretConstants.kHoodMinRevs;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        m_angleMotor.getConfigurator().apply(cfg);
        m_angleHomed  = true;
        m_angleHoming = false;
        m_angleMotor.stopMotor();
        System.out.println("[MobileTurret] Angle homing completo.");
    }

    private void completeRotationHoming() {
        m_rotationMotor.setPosition(0.0);
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode                       = NeutralModeValue.Brake;
        cfg.CurrentLimits.SupplyCurrentLimit              = CurrentLimits.kMechanismSupply;
        cfg.CurrentLimits.SupplyCurrentLimitEnable        = CurrentLimits.kMechanismLimitEnable;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kRotationMax;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kRotationMin;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        m_rotationMotor.getConfigurator().apply(cfg);
        m_rotationHomed  = true;
        m_rotationHoming = false;
        m_rotationMotor.stopMotor();
        System.out.println("[MobileTurret] Rotation homing completo.");
    }

    // ─────────────────────────────────────────────────────────────────────────
    // HOMING — ESTADO PÚBLICO
    // ─────────────────────────────────────────────────────────────────────────
    public boolean isAngleHomed()    { return m_angleHomed; }
    public boolean isRotationHomed() { return m_rotationHomed; }
    public boolean isFullyHomed()    { return m_angleHomed && m_rotationHomed; }

    /** Revoluciones actuales del motor del hood (para SuperstructureCommand). */
    public double getAngleRevs() { return m_anglePosSignal.getValueAsDouble(); }

    // ─────────────────────────────────────────────────────────────────────────
    // VISIÓN
    // ─────────────────────────────────────────────────────────────────────────
    public boolean hasTarget()           { return m_hasTarget; }
    public double  getTargetTX()         { return m_cachedTx; }
    public double  getDistanceToTarget() { return m_cachedDistance; }

    public double getAutoRotationVoltage() {
        if (!m_hasTarget) return 0.0;
        double volts    = m_cachedTx * VisionConstants.kTrackingP * 12.0;
        double maxVolts = MobileTurretConstants.kRotationSpeed * 12.0;
        return Math.max(-maxVolts, Math.min(maxVolts, volts));
    }

    public double getAutoAngleVoltage() {
        if (!m_hasTarget || !m_angleHomed) return 0.0;
        double currentRevs = m_anglePosSignal.getValueAsDouble();
        double targetDeg   = ShootingConfigs.kShotMap.get(m_cachedDistance);
        double targetRevs  = (targetDeg / 360.0) * MobileTurretConstants.kGearRatio;
        double error       = targetRevs - currentRevs;
        double maxVolts    = MobileTurretConstants.kAngleSpeed * 12.0;
        return Math.max(-maxVolts, Math.min(maxVolts, error * maxVolts));
    }

    // ─────────────────────────────────────────────────────────────────────────
    // FACTORÍA DE COMANDOS
    // ─────────────────────────────────────────────────────────────────────────

    public Command homeAngleCommand() {
        return this.runOnce(() -> { m_angleHomed = false; m_angleHoming = true;
            System.out.println("[MobileTurret] Iniciando homing de ángulo..."); })
        .andThen(
            this.run(() -> setAngleVoltage(kAngleHomingVolts))
                .until(() -> { m_angleCurrentSignal.refresh();
                    return m_angleCurrentSignal.getValueAsDouble() > kAngleHomingCurrentThreshold; })
        )
        .andThen(this.runOnce(this::completeAngleHoming));
    }

    public Command homeRotationCommand() {
        return this.runOnce(() -> { m_rotationHomed = false; m_rotationHoming = true;
            System.out.println("[MobileTurret] Iniciando homing de rotación..."); })
        .andThen(
            this.run(() -> setRotationVoltage(kRotationHomingVolts))
                .until(() -> { m_rotationCurrentSignal.refresh();
                    return m_rotationCurrentSignal.getValueAsDouble() > kRotationHomingCurrentThreshold; })
        )
        .andThen(this.runOnce(this::completeRotationHoming));
    }

    /** Homing completo: ángulo primero, luego rotación. */
    public Command homeAllCommand() {
        return homeAngleCommand().andThen(homeRotationCommand());
    }

    /** Control manual. Motores bloqueados si no están homeados. */
    public Command runManualCommand(
            DoubleSupplier angleVolts,
            DoubleSupplier rotationVolts,
            BooleanSupplier shoot) {
        return this.run(() -> {
            setAngleVoltage(m_angleHomed    ? angleVolts.getAsDouble() * 12.0    : 0.0);
            setRotationVoltage(m_rotationHomed ? rotationVolts.getAsDouble() * 12.0 : 0.0);
            setShooterVoltage(shoot.getAsBoolean() ? MobileTurretConstants.kShooterSpeed * 12.0 : 0.0);
        }).finallyDo(interrupted -> stop());
    }

    /** Auto-apuntado con Limelight. Requiere homing completo. */
    public Command runAutoAimCommand(BooleanSupplier shoot) {
        return this.run(() -> {
            if (!isFullyHomed()) return;
            if (m_hasTarget) {
                setRotationVoltage(getAutoRotationVoltage());
                setAngleVoltage(getAutoAngleVoltage());
            } else {
                setRotationVoltage(0);
                setAngleVoltage(0);
            }
            setShooterVoltage(MobileTurretConstants.kShooterSpeed * 12.0);
        }).finallyDo(interrupted -> stop());
    }

    /** Espina shooter y apunta, sin disparar. */
    public Command runSpinUpCommand() {
        return this.run(() -> {
            setShooterVoltage(MobileTurretConstants.kShooterSpeed * 12.0);
            if (isFullyHomed() && m_hasTarget) {
                setRotationVoltage(getAutoRotationVoltage());
                setAngleVoltage(getAutoAngleVoltage());
            }
        }).finallyDo(interrupted -> stop());
    }

    // ─────────────────────────────────────────────────────────────────────────
    // PERIODIC
    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void periodic() {
        m_anglePosSignal.refresh();
        m_angleCurrentSignal.refresh();
        m_shooterVelSignal.refresh();
        m_shooterCurrentSignal.refresh();
        m_rotationPosSignal.refresh();
        m_rotationCurrentSignal.refresh();

        m_hasTarget = LimelightHelpers.getTV(VisionConstants.limelightName);
        m_cachedTx  = LimelightHelpers.getTX(VisionConstants.limelightName);
        if (m_hasTarget) {
            double ty             = LimelightHelpers.getTY(VisionConstants.limelightName);
            double angleToGoalRad = Math.toRadians(VisionConstants.kMountAngleDeg + ty);
            m_cachedDistance      = (VisionConstants.kTargetHeightMeters -
                                    (VisionConstants.kMountAngleDeg * 0.0174533))
                                    / Math.tan(angleToGoalRad);
        } else {
            m_cachedDistance = 0.0;
        }

        SmartDashboard.putNumber("MobileTurret/Angle Revs",       m_anglePosSignal.getValueAsDouble());
        SmartDashboard.putNumber("MobileTurret/Angle Amps",       m_angleCurrentSignal.getValueAsDouble());
        SmartDashboard.putNumber("MobileTurret/Shooter RPS",      m_shooterVelSignal.getValueAsDouble());
        SmartDashboard.putNumber("MobileTurret/Shooter Amps",     m_shooterCurrentSignal.getValueAsDouble());
        SmartDashboard.putNumber("MobileTurret/Rotation Revs",    m_rotationPosSignal.getValueAsDouble());
        SmartDashboard.putNumber("MobileTurret/Rotation Amps",    m_rotationCurrentSignal.getValueAsDouble());
        SmartDashboard.putBoolean("MobileTurret/Angle Homed",     m_angleHomed);
        SmartDashboard.putBoolean("MobileTurret/Rotation Homed",  m_rotationHomed);
        SmartDashboard.putBoolean("MobileTurret/Fully Homed",     isFullyHomed());
        SmartDashboard.putBoolean("MobileTurret/Has Target",      m_hasTarget);
        SmartDashboard.putNumber("MobileTurret/Target TX",        m_cachedTx);
        SmartDashboard.putNumber("MobileTurret/Distance (m)",     m_cachedDistance);
        SmartDashboard.putNumber("MobileTurret/Ideal Angle",      m_hasTarget ? ShootingConfigs.kShotMap.get(m_cachedDistance) : 0.0);
    }
}