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
import frc.robot.Constants.FixedTurretConstants; // ← mayúscula F corregida
import frc.robot.Constants.ShootingConfigs;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class fixedTurretSubsystem extends SubsystemBase {

    // ─────────────────────────────────────────────────────────────────────────
    // POSICIONES DEL HOOD (en revoluciones del motor)
    // Hood range viene de FixedTurretConstants.kHoodMinRevs / kHoodMaxRevs (15°-55°)
    // ─────────────────────────────────────────────────────────────────────────

    private static final double kAngleHomingVolts            = -1.5;  // Negativo = bajar hacia el tope
    private static final double kAngleHomingCurrentThreshold =  8.0;  // Amps

    // ─────────────────────────────────────────────────────────────────────────
    // LÍMITES DE CORRIENTE
    // ─────────────────────────────────────────────────────────────────────────
    // ─────────────────────────────────────────────────────────────────────────
    // MOTORES
    // ─────────────────────────────────────────────────────────────────────────
    private final TalonFX m_angleMotor   = new TalonFX(FixedTurretConstants.kAngleMotorId);
    private final TalonFX m_shooterMotor = new TalonFX(FixedTurretConstants.kShooterMotorId);

    // ─────────────────────────────────────────────────────────────────────────
    // SEÑALES DE MONITOREO
    // ─────────────────────────────────────────────────────────────────────────
    private final StatusSignal<edu.wpi.first.units.measure.Angle>           m_anglePosSignal    = m_angleMotor.getPosition();
    private final StatusSignal<edu.wpi.first.units.measure.Current>         m_angleCurrentSignal = m_angleMotor.getSupplyCurrent();
    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_shooterVelSignal  = m_shooterMotor.getVelocity();
    private final StatusSignal<edu.wpi.first.units.measure.Current>         m_shooterCurrentSignal = m_shooterMotor.getSupplyCurrent();

    // ─────────────────────────────────────────────────────────────────────────
    // ESTADO INTERNO — HOMING
    // ─────────────────────────────────────────────────────────────────────────
    private boolean m_isHomed  = false;
    private boolean m_isHoming = false;

    // ─────────────────────────────────────────────────────────────────────────
    // ESTADO INTERNO — VISIÓN
    // ─────────────────────────────────────────────────────────────────────────
    private double  m_cachedTx       = 0.0;
    private double  m_cachedDistance = 0.0;
    private boolean m_hasTarget      = false;

    // ─────────────────────────────────────────────────────────────────────────
    // CONSTRUCTOR
    // ─────────────────────────────────────────────────────────────────────────
    public fixedTurretSubsystem() {
        // Angle motor
        TalonFXConfiguration angleConfig = new TalonFXConfiguration();
        angleConfig.MotorOutput.NeutralMode                       = NeutralModeValue.Brake;
        angleConfig.CurrentLimits.SupplyCurrentLimit              = CurrentLimits.kMechanismSupply;
        angleConfig.CurrentLimits.SupplyCurrentLimitEnable        = CurrentLimits.kMechanismLimitEnable;
        angleConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FixedTurretConstants.kHoodMaxRevs;
        angleConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = FixedTurretConstants.kHoodMinRevs;
        angleConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable    = false; // OFF hasta homear
        angleConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable    = false;
        m_angleMotor.getConfigurator().apply(angleConfig);

        // Shooter motor
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.NeutralMode              = NeutralModeValue.Coast; // Coast para no frenar en seco
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = CurrentLimits.kNoLimitEnable;
        m_shooterMotor.getConfigurator().apply(shooterConfig);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // MÉTODOS DE ACCIÓN
    // ─────────────────────────────────────────────────────────────────────────
    public void setAngleVoltage(double volts)   { m_angleMotor.setControl(new VoltageOut(volts)); }
    public void setShooterVoltage(double volts) { m_shooterMotor.setControl(new VoltageOut(volts)); }

    public void stop() {
        m_angleMotor.stopMotor();
        m_shooterMotor.stopMotor();
    }

    public boolean isHomed() { return m_isHomed; }

    /** Revoluciones actuales del motor del hood (para SuperstructureCommand). */
    public double getAngleRevs() { return m_anglePosSignal.getValueAsDouble(); }

    // ─────────────────────────────────────────────────────────────────────────
    // HOMING — HELPER PRIVADO
    // ─────────────────────────────────────────────────────────────────────────
    private void completeHoming() {
        m_angleMotor.setPosition(0.0);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode                       = NeutralModeValue.Brake;
        cfg.CurrentLimits.SupplyCurrentLimit              = CurrentLimits.kMechanismSupply;
        cfg.CurrentLimits.SupplyCurrentLimitEnable        = CurrentLimits.kMechanismLimitEnable;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = FixedTurretConstants.kHoodMaxRevs;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = FixedTurretConstants.kHoodMinRevs;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        m_angleMotor.getConfigurator().apply(cfg);

        m_isHomed  = true;
        m_isHoming = false;
        m_angleMotor.stopMotor();
        System.out.println("[FixedTurret] Homing completo. Encoder en 0.");
    }

    // ─────────────────────────────────────────────────────────────────────────
    // VISIÓN
    // ─────────────────────────────────────────────────────────────────────────
    public boolean hasTarget()           { return m_hasTarget; }
    public double  getDistanceToTarget() { return m_cachedDistance; }

    public double getAutoAngleVoltage() {
        if (!m_hasTarget || !m_isHomed) return 0.0;
        double currentRevs = m_anglePosSignal.getValueAsDouble();
        double targetDeg   = ShootingConfigs.kShotMap.get(m_cachedDistance);
        double targetRevs  = (targetDeg / 360.0) * FixedTurretConstants.kGearRatio;
        double error       = targetRevs - currentRevs;
        double maxVolts    = FixedTurretConstants.kAngleSpeed * 12.0;
        return Math.max(-maxVolts, Math.min(maxVolts, error * maxVolts));
    }

    // ─────────────────────────────────────────────────────────────────────────
    // FACTORÍA DE COMANDOS
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * HOMING del hood.
     */
    public Command homingCommand() {
        return this.runOnce(() -> {
            m_isHomed  = false;
            m_isHoming = true;
            System.out.println("[FixedTurret] Iniciando homing...");
        })
        .andThen(
            this.run(() -> setAngleVoltage(kAngleHomingVolts))
                .until(() -> {
                    m_angleCurrentSignal.refresh();
                    return m_angleCurrentSignal.getValueAsDouble() > kAngleHomingCurrentThreshold;
                })
        )
        .andThen(this.runOnce(this::completeHoming));
    }

    /**
     * CONTROL MANUAL: joystick mueve el ángulo, botón dispara.
     * El ángulo no se mueve si no está homeado.
     */
    public Command runManualCommand(DoubleSupplier angleVolts, BooleanSupplier shoot) {
        return this.run(() -> {
            setAngleVoltage(m_isHomed ? angleVolts.getAsDouble() * 12.0 : 0.0);
            setShooterVoltage(shoot.getAsBoolean() ? FixedTurretConstants.kShooterSpeed * 12.0 : 0.0);
        }).finallyDo(interrupted -> stop());
    }

    /**
     * AUTO-AIM con Limelight: ajusta ángulo del hood según distancia al target.
     */
    public Command runAutoAimCommand(BooleanSupplier shoot) {
        return this.run(() -> {
            if (m_isHomed) setAngleVoltage(getAutoAngleVoltage());
            setShooterVoltage(FixedTurretConstants.kShooterSpeed * 12.0);
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

        // Limelight
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

        SmartDashboard.putNumber("FixedTurret/Angle Revs",      m_anglePosSignal.getValueAsDouble());
        SmartDashboard.putNumber("FixedTurret/Angle Amps",      m_angleCurrentSignal.getValueAsDouble());
        SmartDashboard.putNumber("FixedTurret/Shooter RPS",     m_shooterVelSignal.getValueAsDouble());
        SmartDashboard.putNumber("FixedTurret/Shooter Amps",    m_shooterCurrentSignal.getValueAsDouble());
        SmartDashboard.putBoolean("FixedTurret/Is Homed",       m_isHomed);
        SmartDashboard.putBoolean("FixedTurret/Has Target",     m_hasTarget);
        SmartDashboard.putNumber("FixedTurret/Distance (m)",    m_cachedDistance);
        SmartDashboard.putNumber("FixedTurret/TX",              m_cachedTx);
        SmartDashboard.putNumber("FixedTurret/Ideal Angle",     m_hasTarget ? ShootingConfigs.kShotMap.get(m_cachedDistance) : 0.0);
    }
}