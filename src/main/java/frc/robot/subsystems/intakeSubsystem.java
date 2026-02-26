package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.DutyCycleOut;
import frc.robot.Constants.IntakeConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.BooleanSupplier;

public class intakeSubsystem extends SubsystemBase {

  private final TalonFX IntakeMotor = new TalonFX(IntakeConstants.kIntakeMotor);
  private final TalonFX ExtensorMotor = new TalonFX(IntakeConstants.kExtensorMotor);

  public intakeSubsystem() {
    // Constructor
  }

  public void intake(boolean intakeIN, boolean outakeOUT) {
    if (intakeIN) {
      IntakeMotor.setControl(new DutyCycleOut(IntakeConstants.kIntakeSpeedPositive));
    } else if (outakeOUT) {
      IntakeMotor.setControl(new DutyCycleOut(IntakeConstants.kIntakeSpeedNegative));
    } else {
      IntakeMotor.setControl(new DutyCycleOut(0.0));
    }
  }

  public void Extensor(boolean extensorIN, boolean extensorOUT) {
    if (extensorIN) {
      ExtensorMotor.setControl(new DutyCycleOut(IntakeConstants.kExtensorSpeedPositive));
    } else if (extensorOUT) {
      ExtensorMotor.setControl(new DutyCycleOut(IntakeConstants.kExtensorSpeedNegative));
    } else {
      ExtensorMotor.setControl(new DutyCycleOut(0.0));
    }
  }

  public void stop() {
    IntakeMotor.setControl(new DutyCycleOut(0.0));
    ExtensorMotor.setControl(new DutyCycleOut(0.0));
  }

  // Command factory
  public Command runIntakeExtensorCommand(BooleanSupplier intakeIn, BooleanSupplier intakeOut,
      BooleanSupplier extensorIn, BooleanSupplier extensorOut) {

    return this.run(() -> {

      intake(intakeIn.getAsBoolean(), intakeOut.getAsBoolean()); // Controls intake

      Extensor(extensorIn.getAsBoolean(), extensorOut.getAsBoolean()); // Controls extensor

    }).finallyDo(() -> {
      stop(); // Stop motors
    });

  }

}