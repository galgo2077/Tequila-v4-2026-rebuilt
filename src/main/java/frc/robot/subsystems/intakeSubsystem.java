package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
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

  // datos del motor

  // ANGLE MOTOR FIXED

  // asi obtenemos datos del motor
  private final StatusSignal<AngularVelocity> velocityIntakeMotor = IntakeMotor.getVelocity();
  private final StatusSignal<Angle> positionIntakeMotor = IntakeMotor.getPosition();
  private final StatusSignal<Current> currentIntakeMotor = IntakeMotor.getSupplyCurrent();

  // SHOOTER MOTOR FIXED

  // asi obtenemos datos del motor
  private final StatusSignal<AngularVelocity> velocityExtensorMotor = ExtensorMotor.getVelocity();
  private final StatusSignal<Angle> positionExtensorMotor = ExtensorMotor.getPosition();
  private final StatusSignal<Current> currentExtensorMotor = ExtensorMotor.getSupplyCurrent();

  @Override
  public void periodic() {

    velocityIntakeMotor.refresh();
    positionIntakeMotor.refresh();
    currentIntakeMotor.refresh();

    SmartDashboard.putNumber("Intake Motor Velocity", velocityIntakeMotor.getValueAsDouble());
    SmartDashboard.putNumber("Intake Motor Position", positionIntakeMotor.getValueAsDouble());
    SmartDashboard.putNumber("Intake Motor Current", currentIntakeMotor.getValueAsDouble());

    velocityExtensorMotor.refresh();
    positionExtensorMotor.refresh();
    currentExtensorMotor.refresh();

    SmartDashboard.putNumber("Extensor Motor Velocity", velocityExtensorMotor.getValueAsDouble());
    SmartDashboard.putNumber("Extensor Motor Position", positionExtensorMotor.getValueAsDouble());
    SmartDashboard.putNumber("Extensor Motor Current", currentExtensorMotor.getValueAsDouble());

  }
}