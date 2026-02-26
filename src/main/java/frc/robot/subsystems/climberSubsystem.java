package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.DutyCycleOut;
import frc.robot.Constants.climberConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.BooleanSupplier;

public class climberSubsystem extends SubsystemBase {

  private final TalonFX climberMotor = new TalonFX(climberConstants.kClimberMotor);

  public climberSubsystem() {
    // Constructor
  }

  public void climber(boolean climberUP, boolean climberDOWN) {
    if (climberUP) {
      climberMotor.setControl(new DutyCycleOut(climberConstants.kClimberSpeedPositive));
    } else if (climberDOWN) {
      climberMotor.setControl(new DutyCycleOut(climberConstants.kClimberSpeedNegative));
    } else {
      climberMotor.setControl(new DutyCycleOut(0.0));
    }
  }

  public void stop() {
    climberMotor.setControl(new DutyCycleOut(0.0));
  }

  // Command factory
  public Command runClimberCommand(BooleanSupplier climberUP, BooleanSupplier climberDOWN) {

    return this.run(() -> {

      climber(climberUP.getAsBoolean(), climberDOWN.getAsBoolean()); // Controls climber

    }).finallyDo(() -> {
      stop(); // Stop motors
    });

  }

}
