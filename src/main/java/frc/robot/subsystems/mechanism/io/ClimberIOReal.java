package frc.robot.subsystems.mechanism.io;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CurrentLimits;

public class ClimberIOReal implements ClimberIO {

    private final TalonFX m_motor = new TalonFX(ClimberConstants.kClimberMotorId, "rio");

    private final StatusSignal<Double>  m_position;
    private final StatusSignal<Double>  m_velocity;
    private final StatusSignal<Double>  m_voltage;
    private final StatusSignal<Current> m_supplyCurrent;
    private final StatusSignal<Double>  m_temp;

    private final VoltageOut m_voltageRequest = new VoltageOut(0).withEnableFOC(false);

    public ClimberIOReal() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // Current limit
        cfg.CurrentLimits.SupplyCurrentLimitEnable = CurrentLimits.kMechanismLimitEnable;
        cfg.CurrentLimits.SupplyCurrentLimit       = CurrentLimits.kMechanismSupply;

        // Neutral mode
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_motor.getConfigurator().apply(cfg);

        m_position      = m_motor.getPosition();
        m_velocity      = m_motor.getVelocity();
        m_voltage       = m_motor.getMotorVoltage();
        m_supplyCurrent = m_motor.getSupplyCurrent();
        m_temp          = m_motor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
                m_position, m_velocity, m_voltage, m_supplyCurrent, m_temp);
        m_motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.motorConnected    = m_motor.isConnected();
        inputs.positionRevs      = m_position.refresh().getValueAsDouble();
        inputs.velocityRPM       = m_velocity.refresh().getValueAsDouble() * 60.0;
        inputs.appliedVolts      = m_voltage.refresh().getValueAsDouble();
        inputs.supplyCurrentAmps = m_supplyCurrent.refresh().getValueAsDouble();
        inputs.currentAmps       = inputs.supplyCurrentAmps; // keep alias in sync
        inputs.tempCelsius       = m_temp.refresh().getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setControl(m_voltageRequest.withOutput(volts));
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    @Override
    public void resetClimberEncoder() {
        m_motor.setPosition(0.0);
    }

    @Override
    public void enableClimberSoftLimits(boolean forwardEnable, boolean reverseEnable) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        m_motor.getConfigurator().refresh(cfg);
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = forwardEnable;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverseEnable;
        m_motor.getConfigurator().apply(cfg);
    }
}