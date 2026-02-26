package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.DutyCycleOut;
import frc.robot.Constants.FeederConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.BooleanSupplier;

public class feederSubsystem extends SubsystemBase {

    private final TalonFX feederMotor = new TalonFX(FeederConstants.kFeederMotor);

    public feederSubsystem() {
        // Constructor
    }

    // Este le pasa las pelotas al indexer
    public void feeder(boolean feederIN, boolean feederOUT) {

        if (feederIN) {
            feederMotor.setControl(new DutyCycleOut(0.9));
        } else if (feederOUT) {
            feederMotor.setControl(new DutyCycleOut(-0.9));
        } else {
            feederMotor.setControl(new DutyCycleOut(0.0));
        }
    }

    public void stop() {
        feederMotor.setControl(new DutyCycleOut(0.0));
    }

    /// Aca empieza Mi Comando

    public Command runFeederCommand(BooleanSupplier feederIN, BooleanSupplier feederOUT) {

        return this.run(() -> {

            feeder(feederIN.getAsBoolean(), feederOUT.getAsBoolean());// se encarga del feeder

        }).finallyDo(() -> {
            stop();// para que se paren
        });

    }

}