package frc.robot.subsystems.mechanism;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.mechanism.io.ClimberIO;
import frc.robot.subsystems.mechanism.io.ClimberIO.ClimberIOInputs;

import java.util.function.DoubleSupplier;

public class climberSubsystem extends SubsystemBase {

    // ── IO layer ──────────────────────────────────────────────────────────────
    private final ClimberIO io;
    private final ClimberIOInputs inputs = new ClimberIOInputs();

    // ── Posiciones (revs del motor) ───────────────────────────────────────────
    private static final double kClimberMin              = 0.0;   // tope abajo = home
    private static final double kClimberMax              = 50.0;  // completamente extendido (medir)
    private static final double kHomingVoltage           = -1.5;  // negativo = bajar
    private static final double kHomingCurrentThreshold  = 15.0;  // Amps

    // ── Estado interno ────────────────────────────────────────────────────────
    private boolean m_isHomed  = false;
    private boolean m_isHoming = false;

    // ── Constructor ───────────────────────────────────────────────────────────
    public climberSubsystem(ClimberIO io) {
        this.io = io;
    }

    // ── Métodos de acción ─────────────────────────────────────────────────────
    public void setClimberVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void stop() {
        io.stop();
    }

    public boolean isHomed() {
        return m_isHomed;
    }

    // ── Homing — helper privado ───────────────────────────────────────────────
    private void completeHoming() {
        io.resetClimberEncoder();
        io.enableClimberSoftLimits(true, true);
        m_isHomed  = true;
        m_isHoming = false;
        io.stop();
        System.out.println("[Climber] Homing completo. Encoder en 0.");
    }

    // ── Factoría de comandos ──────────────────────────────────────────────────

    /** HOMING: Baja al tope mecánico para encontrar el cero. */
    public Command homingCommand() {
        return this.runOnce(() -> {
            m_isHomed  = false;
            m_isHoming = true;
            System.out.println("[Climber] Iniciando homing...");
        })
        .andThen(
            this.run(() -> setClimberVoltage(kHomingVoltage))
                .until(() -> inputs.currentAmps > kHomingCurrentThreshold)
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

    // ── Periodic ──────────────────────────────────────────────────────────────
    @Override
    public void periodic() {
        io.updateInputs(inputs);

        SmartDashboard.putNumber("Climber/Position Revs", inputs.positionRevs);
        SmartDashboard.putNumber("Climber/Current Amps",  inputs.currentAmps);
        SmartDashboard.putBoolean("Climber/Is Homed",     m_isHomed);
        SmartDashboard.putBoolean("Climber/Is Homing",    m_isHoming);
    }
}