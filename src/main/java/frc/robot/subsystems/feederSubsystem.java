package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import frc.robot.Constants.FeederConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.BooleanSupplier;

public class feederSubsystem extends SubsystemBase {

    private final TalonFX feederMotor = new TalonFX(FeederConstants.kFeederMotor);

    public feederSubsystem() {
        // Constructor
    }

    // Passes balls to indexer
    public void feeder(boolean feederIN, boolean feederOUT) {

        if (feederIN) {
            feederMotor.setControl(new DutyCycleOut(FeederConstants.kFeederSpeedPositive));
        } else if (feederOUT) {
            feederMotor.setControl(new DutyCycleOut(FeederConstants.kFeederSpeedNegative));
        } else {
            feederMotor.setControl(new DutyCycleOut(0.0));
        }
    }

    public void stop() {
        feederMotor.setControl(new DutyCycleOut(0.0));
    }

    // Command factory
    public Command runFeederCommand(BooleanSupplier feederIN, BooleanSupplier feederOUT) {

        return this.run(() -> {

            feeder(feederIN.getAsBoolean(), feederOUT.getAsBoolean()); // Controls feeder

        }).finallyDo(() -> {
            stop(); // Stop motors
        });

    }

    // datos del motor

    // asi obtenemos datos del motor
    private final StatusSignal<AngularVelocity> velocity = feederMotor.getVelocity();
    private final StatusSignal<Angle> position = feederMotor.getPosition();
    private final StatusSignal<Current> current = feederMotor.getSupplyCurrent();

    @Override
    public void periodic() {

        velocity.refresh();
        position.refresh();
        current.refresh();

        SmartDashboard.putNumber("Feeder Velocity", velocity.getValueAsDouble());
        SmartDashboard.putNumber("Feeder Position", position.getValueAsDouble());
        SmartDashboard.putNumber("Feeder Current", current.getValueAsDouble());
    }

}