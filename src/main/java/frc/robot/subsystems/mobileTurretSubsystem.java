package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

}