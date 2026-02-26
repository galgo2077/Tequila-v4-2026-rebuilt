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

    // Passes balls to indexer
    public void roller(boolean rollerIN, boolean rollerOUT) {

        if (rollerIN) {
            rollerMotor.setControl(new DutyCycleOut(0.9));
        } else if (rollerOUT) {
            rollerMotor.setControl(new DutyCycleOut(-0.9));
        } else {
            rollerMotor.setControl(new DutyCycleOut(0.0));
        }
    }

    // Indexer control
    public void index(boolean RightRollerIN, boolean RightRollerOUT, boolean LeftRollerIN, boolean LeftRollerOUT) {

        // Right motor
        if (RightRollerIN) {
            indexerMotorRight.setControl(new DutyCycleOut(0.9));
        } else if (RightRollerOUT) {
            indexerMotorRight.setControl(new DutyCycleOut(-0.9));
        } else {
            indexerMotorRight.setControl(new DutyCycleOut(0.0));
        }

        // Left motor
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

    // Command factory
    public Command runIndexerCommand(BooleanSupplier rollerIN, BooleanSupplier rollerOUT, BooleanSupplier RightRollerIN,
            BooleanSupplier RightRollerOUT, BooleanSupplier LeftRollerIN, BooleanSupplier LeftRollerOUT) {

        return this.run(() -> {

            roller(rollerIN.getAsBoolean(), rollerOUT.getAsBoolean()); // Controls roller

            index(RightRollerIN.getAsBoolean(), RightRollerOUT.getAsBoolean(), LeftRollerIN.getAsBoolean(),
                    LeftRollerOUT.getAsBoolean()); // Controls indexer

        }).finallyDo(() -> {
            stop(); // Stop motors
        });

    }

}