package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
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
            rollerMotor.setControl(new DutyCycleOut(IndexerConstants.kRollerSpeedPositive));
        } else if (rollerOUT) {
            rollerMotor.setControl(new DutyCycleOut(IndexerConstants.kRollerSpeedNegative));
        } else {
            rollerMotor.setControl(new DutyCycleOut(0.0));
        }
    }

    // Indexer control
    public void index(boolean RightRollerIN, boolean RightRollerOUT, boolean LeftRollerIN, boolean LeftRollerOUT) {

        // Right motor
        if (RightRollerIN) {
            indexerMotorRight.setControl(new DutyCycleOut(IndexerConstants.kIndexerSpeedPositive));
        } else if (RightRollerOUT) {
            indexerMotorRight.setControl(new DutyCycleOut(IndexerConstants.kIndexerSpeedNegative));
        } else {
            indexerMotorRight.setControl(new DutyCycleOut(0.0));
        }

        // Left motor
        if (LeftRollerIN) {
            indexerMotorLeft.setControl(new DutyCycleOut(IndexerConstants.kIndexerSpeedPositive));
        } else if (LeftRollerOUT) {
            indexerMotorLeft.setControl(new DutyCycleOut(IndexerConstants.kIndexerSpeedNegative));
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

    // roller motor

    // asi obtenemos datos del motor
    private final StatusSignal<AngularVelocity> velocityRollerMotor = rollerMotor.getVelocity();
    private final StatusSignal<Angle> positionRollerMotor = rollerMotor.getPosition();
    private final StatusSignal<Current> currentRollerMotor = rollerMotor.getSupplyCurrent();

    // indexer motor right

    // asi obtenemos datos del motor
    private final StatusSignal<AngularVelocity> velocityIndexerMotorRight = indexerMotorRight.getVelocity();
    private final StatusSignal<Angle> positionIndexerMotorRight = indexerMotorRight.getPosition();
    private final StatusSignal<Current> currentIndexerMotorRight = indexerMotorRight.getSupplyCurrent();

    // indexer motor left

    private final StatusSignal<AngularVelocity> velocityIndexerMotorLeft = indexerMotorLeft.getVelocity();
    private final StatusSignal<Angle> positionIndexerMotorLeft = indexerMotorLeft.getPosition();
    private final StatusSignal<Current> currentIndexerMotorLeft = indexerMotorLeft.getSupplyCurrent();

    @Override
    public void periodic() {

        velocityRollerMotor.refresh();
        positionRollerMotor.refresh();
        currentRollerMotor.refresh();

        SmartDashboard.putNumber("Roller Motor Velocity", velocityRollerMotor.getValueAsDouble());
        SmartDashboard.putNumber("Roller Motor Position", positionRollerMotor.getValueAsDouble());
        SmartDashboard.putNumber("Roller Motor Current", currentRollerMotor.getValueAsDouble());

        velocityIndexerMotorRight.refresh();
        positionIndexerMotorRight.refresh();
        currentIndexerMotorRight.refresh();

        SmartDashboard.putNumber("Indexer Motor Right Velocity", velocityIndexerMotorRight.getValueAsDouble());
        SmartDashboard.putNumber("Indexer Motor Right Position", positionIndexerMotorRight.getValueAsDouble());
        SmartDashboard.putNumber("Indexer Motor Right Current", currentIndexerMotorRight.getValueAsDouble());

        velocityIndexerMotorLeft.refresh();
        positionIndexerMotorLeft.refresh();
        currentIndexerMotorLeft.refresh();

        SmartDashboard.putNumber("Indexer Motor Left Velocity", velocityIndexerMotorLeft.getValueAsDouble());
        SmartDashboard.putNumber("Indexer Motor Left Position", positionIndexerMotorLeft.getValueAsDouble());
        SmartDashboard.putNumber("Indexer Motor Left Current", currentIndexerMotorLeft.getValueAsDouble());

    }

}