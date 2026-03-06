package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.IntakeConstants;

import java.util.function.BooleanSupplier;

public class intakeSubsystem extends SubsystemBase {

    // ─────────────────────────────────────────────────────────────────────────
    // POSICIONES CONOCIDAS DEL EXTENSOR (en revoluciones del motor)
    // Ajusta estos valores después de hacer el homing una vez y medir
    // ─────────────────────────────────────────────────────────────────────────
    private static final double kExtensorRetracted = 0.0;   // Rev — tope mecánico = home
    private static final double kExtensorDeployed  = 15.0;  // Rev — completamente extendido (medir)

    // Voltaje lento para el homing (empuja contra el tope sin dañar)
    private static final double kHomingVoltage     = -1.5;  // Negativo = retraer
    // Umbral de corriente para detectar que tocó el tope (ajustar con pruebas)
    private static final double kHomingCurrentThreshold = 8.0; // Amps

    // ─────────────────────────────────────────────────────────────────────────
    // MOTORES
    // ─────────────────────────────────────────────────────────────────────────
    private final TalonFX m_intakeMotor   = new TalonFX(IntakeConstants.kIntakeMotorId);
    private final TalonFX m_extensorMotor = new TalonFX(IntakeConstants.kExtensorMotorId);

    // ─────────────────────────────────────────────────────────────────────────
    // SEÑALES DE MONITOREO
    // ─────────────────────────────────────────────────────────────────────────
    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_intakeVel       = m_intakeMotor.getVelocity();
    private final StatusSignal<edu.wpi.first.units.measure.Current>         m_intakeCurrent   = m_intakeMotor.getSupplyCurrent();
    private final StatusSignal<edu.wpi.first.units.measure.Current>         m_extensorCurrent = m_extensorMotor.getSupplyCurrent();
    private final StatusSignal<edu.wpi.first.units.measure.Angle>           m_extensorPos     = m_extensorMotor.getPosition();

    // ─────────────────────────────────────────────────────────────────────────
    // ESTADO INTERNO
    // ─────────────────────────────────────────────────────────────────────────
    private boolean m_isHomed    = false; // ¿Ya hicimos homing este boot?
    private boolean m_isHoming   = false; // ¿Estamos en proceso de homing ahora mismo?

    // ─────────────────────────────────────────────────────────────────────────
    // CONSTRUCTOR
    // ─────────────────────────────────────────────────────────────────────────
    public intakeSubsystem() {
        // --- INTAKE ---
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.Inverted = IntakeConstants.kMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        intakeConfig.CurrentLimits.StatorCurrentLimit       = IntakeConstants.kStatorCurrentLimit;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_intakeMotor.getConfigurator().apply(intakeConfig);

        // --- EXTENSOR ---
        TalonFXConfiguration extensorConfig = new TalonFXConfiguration();
        extensorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Soft limits: el motor se niega a ir más allá de estos valores
        // Se activan DESPUÉS del homing cuando ya sabemos el 0 real
        extensorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold  = kExtensorDeployed;
        extensorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold  = kExtensorRetracted;
        extensorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable     = false; // OFF hasta homear
        extensorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable     = false; // OFF hasta homear
        extensorConfig.CurrentLimits.StatorCurrentLimit               = 30;
        extensorConfig.CurrentLimits.StatorCurrentLimitEnable         = true;
        m_extensorMotor.getConfigurator().apply(extensorConfig);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // MÉTODOS DE ACCIÓN BÁSICOS
    // ─────────────────────────────────────────────────────────────────────────
    public void setIntakeVoltage(double volts) {
        m_intakeMotor.setControl(new VoltageOut(volts));
    }

    public void setExtensorVoltage(double volts) {
        m_extensorMotor.setControl(new VoltageOut(volts));
    }

    public void stop() {
        m_intakeMotor.stopMotor();
        m_extensorMotor.stopMotor();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // POSICIÓN DEL EXTENSOR
    // ─────────────────────────────────────────────────────────────────────────

    /** Revoluciones actuales del motor del extensor */
    public double getExtensorPositionRevs() {
        return m_extensorPos.getValueAsDouble();
    }

    /** ¿Ya se hizo el homing? El driver debe esperar esto antes de usar el extensor */
    public boolean isHomed() {
        return m_isHomed;
    }

    /**
     * Llama esto cuando el homing detectó el tope mecánico.
     * Resetea el encoder a 0 y activa los soft limits.
     */
    private void completeHoming() {
        // 1. Decirle al motor que AQUÍ es el cero
        m_extensorMotor.setPosition(0.0);

        // 2. Activar los soft limits ahora que sabemos el rango real
        TalonFXConfiguration updateConfig = new TalonFXConfiguration();
        updateConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kExtensorDeployed;
        updateConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kExtensorRetracted;
        updateConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        updateConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        m_extensorMotor.getConfigurator().apply(updateConfig);

        m_isHomed  = true;
        m_isHoming = false;

        m_extensorMotor.stopMotor();
        System.out.println("[Extensor] Homing completo. Encoder en 0.");
    }

    // ─────────────────────────────────────────────────────────────────────────
    // FACTORÍA DE COMANDOS
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * COMANDO DE HOMING:
     * Retrae el extensor lentamente contra el tope mecánico.
     * Cuando la corriente sube (chocó el tope), resetea el encoder a 0.
     * Corre una sola vez al inicio del match o después de reboot.
     *
     * Úsalo en RobotContainer así:
     *   new JoystickButton(...).onTrue(intakeSubsystem.homingCommand());
     * O en autonomousInit automáticamente.
     */
    public Command homingCommand() {
        return this.runOnce(() -> {
            m_isHomed  = false;
            m_isHoming = true;
            System.out.println("[Extensor] Iniciando homing...");
        })
        .andThen(
            // Empuja hacia el tope a voltaje bajo hasta detectar corriente alta
            this.run(() -> setExtensorVoltage(kHomingVoltage))
                .until(() -> {
                    m_extensorCurrent.refresh();
                    return m_extensorCurrent.getValueAsDouble() > kHomingCurrentThreshold;
                })
        )
        .andThen(this.runOnce(this::completeHoming));
    }

    /**
     * COMANDO MANUAL CON PROTECCIÓN:
     * Si ya está homeado, los soft limits del motor protegen el rango automáticamente.
     * Si NO está homeado, el extensor no se mueve (seguridad).
     */
    public Command runIntakeExtensorCommand(
            BooleanSupplier intakeIn,
            BooleanSupplier extensorIn,
            BooleanSupplier extensorOut) {

        return this.run(() -> {
            // Intake siempre disponible
            setIntakeVoltage(intakeIn.getAsBoolean() ? IntakeConstants.kIntakeSpeed * 12.0 : 0.0);

            // Extensor solo si ya se hizo el homing
            if (!m_isHomed) {
                setExtensorVoltage(0);
                return;
            }

            if (extensorIn.getAsBoolean()) {
                setExtensorVoltage(IntakeConstants.kExtensorSpeed * 12.0);
            } else if (extensorOut.getAsBoolean()) {
                setExtensorVoltage(-IntakeConstants.kExtensorSpeed * 12.0);
            } else {
                setExtensorVoltage(0);
            }
        }).finallyDo(interrupted -> stop());
    }

    /**
     * COMANDO DE POSICIÓN PRESET:
     * Va directamente a retracted o deployed usando voltaje proporcional al error.
     * Útil para botones de "deploy" y "retract" de un solo toque.
     */
    public Command goToPositionCommand(boolean deploy) {
        double targetRevs = deploy ? kExtensorDeployed : kExtensorRetracted;

        return this.run(() -> {
            if (!m_isHomed) return;

            double error     = targetRevs - getExtensorPositionRevs();
            double voltage   = Math.max(-6.0, Math.min(6.0, error * 0.4)); // P simple
            setExtensorVoltage(voltage);
        })
        .until(() -> Math.abs(targetRevs - getExtensorPositionRevs()) < 0.3) // tolerancia 0.3 revs
        .finallyDo(interrupted -> setExtensorVoltage(0));
    }

    // ─────────────────────────────────────────────────────────────────────────
    // PERIODIC
    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void periodic() {
        m_intakeVel.refresh();
        m_intakeCurrent.refresh();
        m_extensorCurrent.refresh();
        m_extensorPos.refresh();

        SmartDashboard.putNumber("Intake/Velocity RPS",    m_intakeVel.getValueAsDouble());
        SmartDashboard.putNumber("Intake/Current Amps",    m_intakeCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Intake/Extensor Amps",   m_extensorCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Intake/Extensor Revs",   getExtensorPositionRevs());
        SmartDashboard.putBoolean("Intake/Is Homed",       m_isHomed);
        SmartDashboard.putBoolean("Intake/Is Homing",      m_isHoming);
    }
}