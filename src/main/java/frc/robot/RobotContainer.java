// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.subsystems.fixedTurretSubsystem;
import frc.robot.subsystems.mobileTurretSubsystem;
import frc.robot.subsystems.climberSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Robot structure declaration, including subsystems, commands, and triggers.
 */
public class RobotContainer {
  // Subsystems and commands
  private final intakeSubsystem m_intakeSubsystem = new intakeSubsystem();
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  private final feederSubsystem m_feederSubsystem = new feederSubsystem();
  private final fixedTurretSubsystem m_fixedTurretSubsystem = new fixedTurretSubsystem();
  private final mobileTurretSubsystem m_mobileTurretSubsystem = new mobileTurretSubsystem();
  private final climberSubsystem m_climberSubsystem = new climberSubsystem();

  // Controller
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerChasis);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.kDriverControllerIntake);

  // Constructor
  public RobotContainer() {
    // Configure the swerve here

    configureBindings();
  }

  // Defines trigger->command mappings
  private void configureBindings() {

    // feeder controls
    m_operatorController.rightBumper().toggleOnTrue(
        m_feederSubsystem.runFeederCommand(() -> true));

    // fixed turret & indexer shooting
    m_operatorController.leftTrigger().whileTrue(
        m_fixedTurretSubsystem.runFixedTurretCommand(() -> false, () -> false, () -> true)
            .andThen(m_indexerSubsystem.runIndexerCommand(() -> false, () -> false, () -> true, () -> false,
                () -> false, () -> false)));

    // mobile turret & indexer shooting
    m_operatorController.rightTrigger().whileTrue(
        m_mobileTurretSubsystem.runMobileTurretCommand(() -> false, () -> false, () -> false, () -> false, () -> true)
            .andThen(m_indexerSubsystem.runIndexerCommand(() -> false, () -> false, () -> false, () -> false,
                () -> true, () -> false)));

    // condicionales para que funcione el indexer de manera opuesta y solo se vallan
    // a un lado
    if (m_operatorController.leftTrigger().getAsBoolean() == true
        && m_operatorController.rightTrigger().getAsBoolean() == false) {

      m_indexerSubsystem.runIndexerCommand(() -> false, () -> false, () -> false, () -> false, () -> false, () -> true);

    }

    if (m_operatorController.rightTrigger().getAsBoolean() == true
        && m_operatorController.leftTrigger().getAsBoolean() == false) {

      m_indexerSubsystem.runIndexerCommand(() -> false, () -> false, () -> false, () -> true, () -> false, () -> false);

    }
    // intake
    m_operatorController.leftBumper().toggleOnTrue(
        m_intakeSubsystem.runIntakeExtensorCommand(() -> true, () -> false, () -> false));

    // extensor
    m_operatorController.y().whileTrue(
        m_intakeSubsystem.runIntakeExtensorCommand(() -> false, () -> true, () -> false));
    m_operatorController.b().whileTrue(
        m_intakeSubsystem.runIntakeExtensorCommand(() -> false, () -> false, () -> true));

    // roller
    m_operatorController.x().whileTrue(
        m_indexerSubsystem.runIndexerCommand(() -> true, () -> false, () -> false, () -> false, () -> false,
            () -> false));

    m_operatorController.a().toggleOnTrue(
        m_indexerSubsystem.runIndexerCommand(() -> true, () -> false, () -> true, () -> false, () -> false,
            () -> false));

    // escalador

    m_operatorController.rightStick().whileTrue(
        m_climberSubsystem.runClimberCommand(() -> -m_operatorController.getRightY()));

    // falta emter angle y asi

  }

  // Returns the autonomous command

  // public Command getAutonomousCommand() {
  // public Command getAutonomousCommand() {
  // return Autos.exampleAuto(m_intakeSubsystem, m_indexerSubsystem,
  // m_feederSubsystem);
  // }
}
