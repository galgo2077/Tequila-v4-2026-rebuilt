package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import frc.robot.Constants.fixedTurretConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.BooleanSupplier;

public class fixedTurretSubsystem extends SubsystemBase {

    private final TalonFX angleMotorFixed = new TalonFX(fixedTurretConstants.kAngleMotorFixed);
    private final TalonFX ShooterFixedMotor = new TalonFX(fixedTurretConstants.kShooterFixedMotor);

    public fixedTurretSubsystem() {
        // Constructor
    }

    public void angle(boolean angleIN, boolean angleOUT) {
        if (angleIN) {
            angleMotorFixed.setControl(new DutyCycleOut(fixedTurretConstants.kAngleSpeedPositiveFixed));
        } else if (angleOUT) {
            angleMotorFixed.setControl(new DutyCycleOut(fixedTurretConstants.kAngleSpeedNegativeFixed));
        } else {
            angleMotorFixed.setControl(new DutyCycleOut(0.0));
        }
    }

    public void ShooterFixed(boolean ShooterFixedIN, boolean ShooterFixedOUT) {
        if (ShooterFixedIN) {
            ShooterFixedMotor.setControl(new DutyCycleOut(fixedTurretConstants.kShooterSpeedPositiveFixed));
        } else if (ShooterFixedOUT) {
            ShooterFixedMotor.setControl(new DutyCycleOut(fixedTurretConstants.kShoterSpeedNegativeFixed));
        } else {
            ShooterFixedMotor.setControl(new DutyCycleOut(0.0));
        }
    }

    public void stop() {
        angleMotorFixed.setControl(new DutyCycleOut(0.0));
        ShooterFixedMotor.setControl(new DutyCycleOut(0.0));
    }

    // Command factory
    public Command runFixedTurretCommand(BooleanSupplier angleIN, BooleanSupplier angleOUT,
            BooleanSupplier ShooterFixedIN, BooleanSupplier ShooterFixedOUT) {

        return this.run(() -> {

            angle(angleIN.getAsBoolean(), angleOUT.getAsBoolean()); // Controls angle

            ShooterFixed(ShooterFixedIN.getAsBoolean(), ShooterFixedOUT.getAsBoolean()); // Controls ShooterFixed

        }).finallyDo(() -> {
            stop(); // Stop motors
        });

    }

    // datos del motor

    // ANGLE MOTOR FIXED

    // asi obtenemos datos del motor
    private final StatusSignal<AngularVelocity> velocityAngleMotorFixed = angleMotorFixed.getVelocity();
    private final StatusSignal<Angle> positionAngleMotorFixed = angleMotorFixed.getPosition();
    private final StatusSignal<Current> currentAngleMotorFixed = angleMotorFixed.getSupplyCurrent();

    // SHOOTER MOTOR FIXED

    // asi obtenemos datos del motor
    private final StatusSignal<AngularVelocity> velocityShooterFixedMotor = ShooterFixedMotor.getVelocity();
    private final StatusSignal<Angle> positionShooterFixedMotor = ShooterFixedMotor.getPosition();
    private final StatusSignal<Current> currentShooterFixedMotor = ShooterFixedMotor.getSupplyCurrent();

    @Override
    public void periodic() {

        velocityAngleMotorFixed.refresh();
        positionAngleMotorFixed.refresh();
        currentAngleMotorFixed.refresh();

        SmartDashboard.putNumber("Angle Motor Fixed Velocity", velocityAngleMotorFixed.getValueAsDouble());
        SmartDashboard.putNumber("Angle Motor Fixed Position", positionAngleMotorFixed.getValueAsDouble());
        SmartDashboard.putNumber("Angle Motor Fixed Current", currentAngleMotorFixed.getValueAsDouble());

        velocityShooterFixedMotor.refresh();
        positionShooterFixedMotor.refresh();
        currentShooterFixedMotor.refresh();

        SmartDashboard.putNumber("Shooter Fixed Velocity", velocityShooterFixedMotor.getValueAsDouble());
        SmartDashboard.putNumber("Shooter Fixed Position", positionShooterFixedMotor.getValueAsDouble());
        SmartDashboard.putNumber("Shooter Fixed Current", currentShooterFixedMotor.getValueAsDouble());

    }

}