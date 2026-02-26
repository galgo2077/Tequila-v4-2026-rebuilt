package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.DutyCycleOut;
import frc.robot.Constants.IndexerConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.BooleanSupplier;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonFX rollerMotor = new TalonFX(IndexerConstants.kRollerMotor);
    private final TalonFX indexerMotorRight = new TalonFX(IndexerConstants.kIndexerMotorRight);
    private final TalonFX indexerMotorLeft = new TalonFX(IndexerConstants.kIndexerMotorLeft);

    public IndexerSubsystem() {
        // Constructor
    }

    // Este le pasa las pelotas al indexer
    public void roller(boolean rollerIN, boolean rollerOUT) {

        if (rollerIN) {
            rollerMotor.setControl(new DutyCycleOut(0.9));
        } else if (rollerOUT) {
            rollerMotor.setControl(new DutyCycleOut(-0.9));
        } else {
            rollerMotor.setControl(new DutyCycleOut(0.0));
        }
    }

    // Este es el indexer
    public void index(boolean RightRollerIN, boolean RightRollerOUT, boolean LeftRollerIN, boolean LeftRollerOUT) {

        // controles para el motor derecho
        if (RightRollerIN) {
            indexerMotorRight.setControl(new DutyCycleOut(0.9));
        } else if (RightRollerOUT) {
            indexerMotorRight.setControl(new DutyCycleOut(-0.9));
        } else {
            indexerMotorRight.setControl(new DutyCycleOut(0.0));
        }

        // controles para el motor izquierdo
        if (LeftRollerIN) {
            indexerMotorLeft.setControl(new DutyCycleOut(0.9));
        } else if (LeftRollerOUT) {
            indexerMotorLeft.setControl(new DutyCycleOut(-0.9));
        } else {
            indexerMotorLeft.setControl(new DutyCycleOut(0.0));
        }
    }

    public void stop() {
        rollerMotor.setControl(new DutyCycleOut(0.0));
        indexerMotorRight.setControl(new DutyCycleOut(0.0));
        indexerMotorLeft.setControl(new DutyCycleOut(0.0));
    }

    /// Aca empieza Mi Comando

    public Command runIndexerCommand(BooleanSupplier rollerIN, BooleanSupplier rollerOUT, BooleanSupplier RightRollerIN,
            BooleanSupplier RightRollerOUT, BooleanSupplier LeftRollerIN, BooleanSupplier LeftRollerOUT) {

        return this.run(() -> {

            roller(rollerIN.getAsBoolean(), rollerOUT.getAsBoolean());// se encarga del roller

            index(RightRollerIN.getAsBoolean(), RightRollerOUT.getAsBoolean(), LeftRollerIN.getAsBoolean(),
                    LeftRollerOUT.getAsBoolean());// se encarga del indexer

        }).finallyDo(() -> {
            stop();// para que se paren
        });

    }

}