package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import frc.robot.Constants.climberConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class climberSubsystem extends SubsystemBase {

  // AdvantageScope publishers

  private final TalonFX climberMotor = new TalonFX(climberConstants.kClimberMotor);

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

  // datos del motor

  // asi obtenemos datos del motor
  private final StatusSignal<AngularVelocity> velocity = climberMotor.getVelocity();
  private final StatusSignal<Angle> position = climberMotor.getPosition();
  private final StatusSignal<Current> current = climberMotor.getSupplyCurrent();

  @Override
  public void periodic() {

    velocity.refresh();
    position.refresh();
    current.refresh();

    SmartDashboard.putNumber("Climber Velocity", velocity.getValueAsDouble());
    SmartDashboard.putNumber("Climber Position", position.getValueAsDouble());
    SmartDashboard.putNumber("Climber Current", current.getValueAsDouble());
  }

}
