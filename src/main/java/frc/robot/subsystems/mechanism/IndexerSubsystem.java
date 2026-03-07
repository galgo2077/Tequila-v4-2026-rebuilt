package frc.robot.subsystems.mechanism;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CurrentLimits;
import frc.robot.Constants.IndexerConstants;
import java.util.function.BooleanSupplier;

public class IndexerSubsystem extends SubsystemBase {

    // ─────────────────────────────────────────────────────────────────────────
    // LÍMITES DE CORRIENTE
    // ─────────────────────────────────────────────────────────────────────────
    // ─────────────────────────────────────────────────────────────────────────
    // MOTORES
    // ─────────────────────────────────────────────────────────────────────────
    private final TalonFX m_rollerMotor        = new TalonFX(IndexerConstants.kRollerMotorId);
    private final TalonFX m_indexerMotorRight  = new TalonFX(IndexerConstants.kMotorRightId);
    private final TalonFX m_indexerMotorLeft   = new TalonFX(IndexerConstants.kMotorLeftId);

    // ─────────────────────────────────────────────────────────────────────────
    // SEÑALES DE MONITOREO
    // ─────────────────────────────────────────────────────────────────────────
    private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> m_rollerVel    = m_rollerMotor.getVelocity();
    private final StatusSignal<edu.wpi.first.units.measure.Current>         m_rightCurrent = m_indexerMotorRight.getSupplyCurrent();
    private final StatusSignal<edu.wpi.first.units.measure.Current>         m_leftCurrent  = m_indexerMotorLeft.getSupplyCurrent();
    private final StatusSignal<edu.wpi.first.units.measure.Current>         m_rollerCurrent = m_rollerMotor.getSupplyCurrent();

    // ─────────────────────────────────────────────────────────────────────────
    // CONSTRUCTOR
    // ─────────────────────────────────────────────────────────────────────────
    public IndexerSubsystem() {
        // Config base compartida
        TalonFXConfiguration baseConfig = new TalonFXConfiguration();
        baseConfig.MotorOutput.NeutralMode              = NeutralModeValue.Brake;
        baseConfig.CurrentLimits.SupplyCurrentLimit     = CurrentLimits.kMechanismSupply;
        baseConfig.CurrentLimits.SupplyCurrentLimitEnable = CurrentLimits.kMechanismLimitEnable;

        // Roller (sin inversión específica)
        m_rollerMotor.getConfigurator().apply(baseConfig);

        // Indexer Derecho
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightConfig.MotorOutput.Inverted    = IndexerConstants.kInvertRight
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        rightConfig.CurrentLimits.SupplyCurrentLimit      = CurrentLimits.kMechanismSupply;
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = CurrentLimits.kMechanismLimitEnable;
        m_indexerMotorRight.getConfigurator().apply(rightConfig);

        // Indexer Izquierdo
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftConfig.MotorOutput.Inverted    = IndexerConstants.kInvertLeft
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        leftConfig.CurrentLimits.SupplyCurrentLimit      = CurrentLimits.kMechanismSupply;
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = CurrentLimits.kMechanismLimitEnable;
        m_indexerMotorLeft.getConfigurator().apply(leftConfig);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // MÉTODOS DE CONTROL
    // ─────────────────────────────────────────────────────────────────────────
    public void setRollerVoltage(double volts) {
        m_rollerMotor.setControl(new VoltageOut(volts));
    }

    public void setIndexerVoltage(double rightVolts, double leftVolts) {
        m_indexerMotorRight.setControl(new VoltageOut(rightVolts));
        m_indexerMotorLeft.setControl(new VoltageOut(leftVolts));
    }

    public void stop() {
        m_rollerMotor.stopMotor();
        m_indexerMotorRight.stopMotor();
        m_indexerMotorLeft.stopMotor();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // FACTORÍA DE COMANDOS
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Control completo del indexer: roller y cintas sincronizadas.
     */
    public Command runIndexerCommand(
            BooleanSupplier rollerIn,
            BooleanSupplier rollerOut,
            BooleanSupplier indexIn,
            BooleanSupplier indexOut) {

        return this.run(() -> {
            // Roller
            if (rollerIn.getAsBoolean()) {
                setRollerVoltage(IndexerConstants.kRollerSpeed * 12.0);
            } else if (rollerOut.getAsBoolean()) {
                setRollerVoltage(-IndexerConstants.kRollerSpeed * 12.0);
            } else {
                setRollerVoltage(0);
            }

            // Cintas (ambas sincronizadas)
            if (indexIn.getAsBoolean()) {
                setIndexerVoltage(
                    IndexerConstants.kIndexerSpeed * 12.0,
                    IndexerConstants.kIndexerSpeed * 12.0);
            } else if (indexOut.getAsBoolean()) {
                setIndexerVoltage(
                    -IndexerConstants.kIndexerSpeed * 12.0,
                    -IndexerConstants.kIndexerSpeed * 12.0);
            } else {
                setIndexerVoltage(0, 0);
            }
        }).finallyDo(interrupted -> stop());
    }

    // ─────────────────────────────────────────────────────────────────────────
    // PERIODIC
    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void periodic() {
        m_rollerVel.refresh();
        m_rightCurrent.refresh();
        m_leftCurrent.refresh();
        m_rollerCurrent.refresh();

        SmartDashboard.putNumber("Indexer/Roller RPS",    m_rollerVel.getValueAsDouble());
        SmartDashboard.putNumber("Indexer/Roller Amps",   m_rollerCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Indexer/Right Amps",    m_rightCurrent.getValueAsDouble());
        SmartDashboard.putNumber("Indexer/Left Amps",     m_leftCurrent.getValueAsDouble());
    }
}