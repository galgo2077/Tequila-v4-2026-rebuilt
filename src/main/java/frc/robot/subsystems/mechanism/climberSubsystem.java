package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ClimberConstants;
import java.util.function.DoubleSupplier;

public class climberSubsystem extends SubsystemBase {

    // ─────────────────────────────────────────────────────────────────────────
    // POSICIONES (en revoluciones del motor)
    // Ajusta kClimberMax después de homear y medir
    // ─────────────────────────────────────────────────────────────────────────
    private static final double kClimberMin                    =  0.0;  // Rev — tope abajo = home
    private static final double kClimberMax                    = 50.0;  // Rev — completamente extendido (medir)
    private static final double kHomingVoltage                 = -1.5;  // Negativo = bajar hacia el tope
    private static final double kHomingCurrentThreshold        = 15.0;  // Amps — más alto porque el climber es pesado

    // ─────────────────────────────────────────────────────────────────────────
    // MOTOR
    // ─────────────────────────────────────────────────────────────────────────
    private final TalonFX m_climberMotor = new TalonFX(ClimberConstants.kClimberMotorId);

    // ─────────────────────────────────────────────────────────────────────────
    // SEÑALES DE MONITOREO
    // ─────────────────────────────────────────────────────────────────────────
    private final StatusSignal<edu.wpi.first.units.measure.Angle>   m_position = m_climberMotor.getPosition();
    private final StatusSignal<edu.wpi.first.units.measure.Current> m_current  = m_climberMotor.getSupplyCurrent();

    // ─────────────────────────────────────────────────────────────────────────
    // ESTADO INTERNO
    // ─────────────────────────────────────────────────────────────────────────
    private boolean m_isHomed  = false;
    private boolean m_isHoming = false;

    // ─────────────────────────────────────────────────────────────────────────
    // CONSTRUCTOR
    // ─────────────────────────────────────────────────────────────────────────
    public climberSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // BRAKE: vital para que el robot se quede colgado al soltar el joystick
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // CORRIENTE: 60A para levantar el peso del robot
        config.CurrentLimits.StatorCurrentLimit       = ClimberConstants.kCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit       = ClimberConstants.kCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Soft limits OFF hasta homear
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kClimberMax;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kClimberMin;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable    = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable    = false;

        m_climberMotor.getConfigurator().apply(config);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // MÉTODOS DE ACCIÓN
    // ─────────────────────────────────────────────────────────────────────────
    public void setClimberVoltage(double volts) {
        m_climberMotor.setControl(new VoltageOut(volts));
    }

    public void stop() {
        m_climberMotor.stopMotor();
    }

    public boolean isHomed() { return m_isHomed; }

    // ─────────────────────────────────────────────────────────────────────────
    // HOMING — HELPER PRIVADO
    // ─────────────────────────────────────────────────────────────────────────
    private void completeHoming() {
        m_climberMotor.setPosition(0.0);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode                       = NeutralModeValue.Brake;
        cfg.CurrentLimits.StatorCurrentLimit              = ClimberConstants.kCurrentLimit;
        cfg.CurrentLimits.StatorCurrentLimitEnable        = true;
        cfg.CurrentLimits.SupplyCurrentLimit              = ClimberConstants.kCurrentLimit;
        cfg.CurrentLimits.SupplyCurrentLimitEnable        = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kClimberMax;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kClimberMin;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        m_climberMotor.getConfigurator().apply(cfg);

        m_isHomed  = true;
        m_isHoming = false;
        m_climberMotor.stopMotor();
        System.out.println("[Climber] Homing completo. Encoder en 0.");
    }

    // ─────────────────────────────────────────────────────────────────────────
    // FACTORÍA DE COMANDOS
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * HOMING: Baja el climber al tope mecánico para encontrar el cero.
     */
    public Command homingCommand() {
        return this.runOnce(() -> {
            m_isHomed  = false;
            m_isHoming = true;
            System.out.println("[Climber] Iniciando homing...");
        })
        .andThen(
            this.run(() -> setClimberVoltage(kHomingVoltage))
                .until(() -> {
                    m_current.refresh();
                    return m_current.getValueAsDouble() > kHomingCurrentThreshold;
                })
        )
        .andThen(this.runOnce(this::completeHoming));
    }

    /**
     * CONTROL MANUAL con joystick.
     * No se mueve si no está homeado (seguridad).
     */
    public Command runClimberCommand(DoubleSupplier speedSupplier) {
        return this.run(() -> {
            if (!m_isHomed) {
                setClimberVoltage(0);
                return;
            }
            double input = speedSupplier.getAsDouble();
            if (Math.abs(input) > 0.1) {
                setClimberVoltage(input * ClimberConstants.kMaxSpeed * 12.0);
            } else {
                setClimberVoltage(0);
            }
        }).finallyDo(interrupted -> stop());
    }

    // ─────────────────────────────────────────────────────────────────────────
    // PERIODIC
    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void periodic() {
        m_position.refresh();
        m_current.refresh();

        SmartDashboard.putNumber("Climber/Position Revs", m_position.getValueAsDouble());
        SmartDashboard.putNumber("Climber/Current Amps",  m_current.getValueAsDouble());
        SmartDashboard.putBoolean("Climber/Is Homed",     m_isHomed);
        SmartDashboard.putBoolean("Climber/Is Homing",    m_isHoming);
    }
}
