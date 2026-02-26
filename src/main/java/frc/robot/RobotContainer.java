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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule exampleMethodCommand on B button press
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  // Returns the autonomous command

  // public Command getAutonomousCommand() {
  // public Command getAutonomousCommand() {
  // return Autos.exampleAuto(m_intakeSubsystem, m_indexerSubsystem,
  // m_feederSubsystem);
  // }
}
