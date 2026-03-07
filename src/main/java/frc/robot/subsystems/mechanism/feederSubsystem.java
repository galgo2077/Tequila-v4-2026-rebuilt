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
import frc.robot.Constants.FeederConstants;
import java.util.function.BooleanSupplier;

public class feederSubsystem extends SubsystemBase {

    // ─────────────────────────────────────────────────────────────────────────
    // LÍMITES DE CORRIENTE
    // ─────────────────────────────────────────────────────────────────────────
    // ─────────────────────────────────────────────────────────────────────────
    // MOTOR
    // ─────────────────────────────────────────────────────────────────────────
    private final TalonFX m_feederMotor = new TalonFX(FeederConstants.kFeederMotorId);

    // ─────────────────────────────────────────────────────────────────────────
    // SEÑALES DE MONITOREO
    // ─────────────────────────────────────────────────────────────────────────
    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_velocity = m_feederMotor.getVelocity();
    private final StatusSignal<edu.wpi.first.units.measure.Current>         m_current  = m_feederMotor.getSupplyCurrent();

    // ─────────────────────────────────────────────────────────────────────────
    // CONSTRUCTOR
    // ─────────────────────────────────────────────────────────────────────────
    public feederSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // COAST: no frenar en seco la pieza al pasar por el feeder
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Stator: protege el motor si la pieza se atasca

        // Supply: protege el cableado y breaker
        config.CurrentLimits.SupplyCurrentLimit       = CurrentLimits.kMechanismSupply;
        config.CurrentLimits.SupplyCurrentLimitEnable = CurrentLimits.kMechanismLimitEnable;

        m_feederMotor.getConfigurator().apply(config);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // MÉTODOS DE ACCIÓN
    // ─────────────────────────────────────────────────────────────────────────
    public void setFeederVoltage(double volts) {
        m_feederMotor.setControl(new VoltageOut(volts));
    }

    public void stop() {
        m_feederMotor.stopMotor();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // FACTORÍA DE COMANDOS
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Activa el feeder mientras el botón esté presionado.
     */
    public Command runFeederCommand(BooleanSupplier feederIn) {
        return this.run(() -> {
            setFeederVoltage(feederIn.getAsBoolean() ? FeederConstants.kFeederSpeed * 12.0 : 0.0);
        }).finallyDo(interrupted -> stop());
    }

    /**
     * Activa el feeder en reversa (expulsar pieza).
     */
    public Command reverseFeederCommand(BooleanSupplier feederOut) {
        return this.run(() -> {
            setFeederVoltage(feederOut.getAsBoolean() ? -FeederConstants.kFeederSpeed * 12.0 : 0.0);
        }).finallyDo(interrupted -> stop());
    }

    // ─────────────────────────────────────────────────────────────────────────
    // PERIODIC
    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void periodic() {
        m_velocity.refresh();
        m_current.refresh();

        SmartDashboard.putNumber("Feeder/Velocity RPS", m_velocity.getValueAsDouble());
        SmartDashboard.putNumber("Feeder/Current Amps", m_current.getValueAsDouble());
    }
}