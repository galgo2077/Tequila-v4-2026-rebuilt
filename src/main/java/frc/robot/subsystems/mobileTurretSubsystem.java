package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import frc.robot.Constants.mobileTurretConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.BooleanSupplier;

public class mobileTurretSubsystem extends SubsystemBase {

    private final TalonFX angleMotorMobile = new TalonFX(mobileTurretConstants.kAngleMotorMobile);
    private final TalonFX ShooterMotorMobile = new TalonFX(mobileTurretConstants.kShooterMotorMobile);
    private final TalonFX TurretMotorMobile = new TalonFX(mobileTurretConstants.kTurretMotorMobile);

    public mobileTurretSubsystem() {
        // Constructor
    }

    public void angle(boolean angleIN, boolean angleOUT) {
        if (angleIN) {
            angleMotorMobile.setControl(new DutyCycleOut(mobileTurretConstants.kAngleSpeedPositiveMobile));
        } else if (angleOUT) {
            angleMotorMobile.setControl(new DutyCycleOut(mobileTurretConstants.kAngleSpeedNegativeMobile));
        } else {
            angleMotorMobile.setControl(new DutyCycleOut(0.0));
        }
    }

    public void ShooterMobile(boolean ShooterMobileIN, boolean ShooterMobileOUT) {
        if (ShooterMobileIN) {
            ShooterMotorMobile.setControl(new DutyCycleOut(mobileTurretConstants.kShooterSpeedPositiveMobile));
        } else if (ShooterMobileOUT) {
            ShooterMotorMobile.setControl(new DutyCycleOut(mobileTurretConstants.kShooterSpeedNegativeMobile));
        } else {
            ShooterMotorMobile.setControl(new DutyCycleOut(0.0));
        }
    }

    public void TurretMobile(boolean TurretMobileIN, boolean TurretMobileOUT) {
        if (TurretMobileIN) {
            TurretMotorMobile.setControl(new DutyCycleOut(mobileTurretConstants.kTurretSpeedPositiveMobile));
        } else if (TurretMobileOUT) {
            TurretMotorMobile.setControl(new DutyCycleOut(mobileTurretConstants.kTurretSpeedNegativeMobile));
        } else {
            TurretMotorMobile.setControl(new DutyCycleOut(0.0));
        }
    }

    public void stop() {
        angleMotorMobile.setControl(new DutyCycleOut(0.0));
        ShooterMotorMobile.setControl(new DutyCycleOut(0.0));
        TurretMotorMobile.setControl(new DutyCycleOut(0.0));
    }

    // Command factory
    public Command runMobileTurretCommand(BooleanSupplier angleIN, BooleanSupplier angleOUT,
            BooleanSupplier ShooterMobileIN, BooleanSupplier ShooterMobileOUT,
            BooleanSupplier TurretMobileIN, BooleanSupplier TurretMobileOUT) {

        return this.run(() -> {

            angle(angleIN.getAsBoolean(), angleOUT.getAsBoolean()); // Controls angle

            ShooterMobile(ShooterMobileIN.getAsBoolean(), ShooterMobileOUT.getAsBoolean()); // Controls ShooterMobile

            TurretMobile(TurretMobileIN.getAsBoolean(), TurretMobileOUT.getAsBoolean()); // Controls TurretMobile

        }).finallyDo(() -> {
            stop(); // Stop motors
        });

    }

    // roller motor

    // asi obtenemos datos del motor
    private final StatusSignal<AngularVelocity> velocityAngleMotorMobile = angleMotorMobile.getVelocity();
    private final StatusSignal<Angle> positionAngleMotorMobile = angleMotorMobile.getPosition();
    private final StatusSignal<Current> currentAngleMotorMobile = angleMotorMobile.getSupplyCurrent();

    // indexer motor right

    // asi obtenemos datos del motor
    private final StatusSignal<AngularVelocity> velocityShooterMotorMobile = ShooterMotorMobile.getVelocity();
    private final StatusSignal<Angle> positionShooterMotorMobile = ShooterMotorMobile.getPosition();
    private final StatusSignal<Current> currentShooterMotorMobile = ShooterMotorMobile.getSupplyCurrent();

    // indexer motor left

    private final StatusSignal<AngularVelocity> velocityTurretMotorMobile = TurretMotorMobile.getVelocity();
    private final StatusSignal<Angle> positionTurretMotorMobile = TurretMotorMobile.getPosition();
    private final StatusSignal<Current> currentTurretMotorMobile = TurretMotorMobile.getSupplyCurrent();

    @Override
    public void periodic() {

        velocityAngleMotorMobile.refresh();
        positionAngleMotorMobile.refresh();
        currentAngleMotorMobile.refresh();

        SmartDashboard.putNumber("Angle Motor Velocity", velocityAngleMotorMobile.getValueAsDouble());
        SmartDashboard.putNumber("Angle Motor Position", positionAngleMotorMobile.getValueAsDouble());
        SmartDashboard.putNumber("Angle Motor Current", currentAngleMotorMobile.getValueAsDouble());

        velocityShooterMotorMobile.refresh();
        positionShooterMotorMobile.refresh();
        currentShooterMotorMobile.refresh();

        SmartDashboard.putNumber("Shooter Motor Velocity", velocityShooterMotorMobile.getValueAsDouble());
        SmartDashboard.putNumber("Shooter Motor Position", positionShooterMotorMobile.getValueAsDouble());
        SmartDashboard.putNumber("Shooter Motor Current", currentShooterMotorMobile.getValueAsDouble());

        velocityTurretMotorMobile.refresh();
        positionTurretMotorMobile.refresh();
        currentTurretMotorMobile.refresh();

        SmartDashboard.putNumber("Turret Motor Velocity", velocityTurretMotorMobile.getValueAsDouble());
        SmartDashboard.putNumber("Turret Motor Position", positionTurretMotorMobile.getValueAsDouble());
        SmartDashboard.putNumber("Turret Motor Current", currentTurretMotorMobile.getValueAsDouble());

    }

}