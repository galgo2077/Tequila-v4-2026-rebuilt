// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.climberSubsystem;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.subsystems.fixedTurretSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.mobileTurretSubsystem;
import frc.robot.subsystems.Mates.angulo_shooter;
import frc.robot.subsystems.Mates.tiempo_angle;

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

  angulo_shooter angulo_shooter = new angulo_shooter();
  tiempo_angle tiempo_angle = new tiempo_angle();

  double tiempo_inicio = tiempo_angle.tiempo_inicio();
  double tiempo_posicionar = tiempo_angle
      .tiempo_angulo_posicionar(angulo_shooter.getrealangle(angulo_shooter.getTargetAngle()));

  // Constructor
  public RobotContainer() {
    // Configure the swerve here

    configureBindings();
    turret_fixed_shooter();
  }

  // Defines trigger->command mappings
  private void configureBindings() {

    // feeder controls
    m_operatorController.rightBumper().toggleOnTrue(
        m_feederSubsystem.runFeederCommand(() -> true));

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

  }

  private void turret_fixed_shooter() {

    // fixed

    // 1. DISPARO: Mientras presionas, corre hasta que se acabe el tiempo de inicio
    m_operatorController.leftTrigger()
        .onTrue(m_fixedTurretSubsystem.runFixedTurretCommand(() -> true, () -> false, () -> false)
            .withTimeout(tiempo_posicionar))
        .whileTrue(m_fixedTurretSubsystem.runFixedTurretCommand(() -> false, () -> false, () -> true)
            .alongWith(m_indexerSubsystem.runIndexerCommand(() -> false, () -> false, () -> true, () -> false,
                () -> false, () -> false))
            .withTimeout(tiempo_inicio));

    // 2. POSICIONAMIENTO: Al SOLTAR el gatillo, se ejecuta el regreso por un tiempo
    // calculado
    m_operatorController.leftTrigger().onFalse(
        m_fixedTurretSubsystem.runFixedTurretCommand(() -> false, () -> true, () -> false)
            .withTimeout(tiempo_inicio) // Se apaga solo al terminar el tiempo
    );

    // mobile
    // mobile turret & indexer shooting
    m_operatorController.rightTrigger()
        .onTrue(m_mobileTurretSubsystem
            .runMobileTurretCommand(() -> true, () -> false, () -> false, () -> false, () -> false)
            .withTimeout(tiempo_posicionar))
        .whileTrue(m_mobileTurretSubsystem
            .runMobileTurretCommand(() -> false, () -> false, () -> false, () -> false, () -> true)
            .alongWith(m_indexerSubsystem.runIndexerCommand(() -> false, () -> false, () -> false, () -> false,
                () -> true, () -> false)));

    m_operatorController.rightTrigger().onFalse(
        m_mobileTurretSubsystem.runMobileTurretCommand(() -> false, () -> true, () -> false, () -> false, () -> false)
            .withTimeout(tiempo_inicio));

    // posicionate angle stock
    m_operatorController.povUp().whileTrue(
        m_fixedTurretSubsystem.runFixedTurretCommand(() -> true, () -> false, () -> false)
            .alongWith(m_mobileTurretSubsystem.runMobileTurretCommand(() -> true, () -> false, () -> false, () -> false,
                () -> false)));

    m_operatorController.povDown().whileTrue(
        m_fixedTurretSubsystem.runFixedTurretCommand(() -> false, () -> true, () -> false)
            .alongWith(m_mobileTurretSubsystem.runMobileTurretCommand(() -> false, () -> true, () -> false, () -> false,
                () -> false)));

    // da el indexer y eso

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

  }

  // Returns the autonomous command

  // public Command getAutonomousCommand() {
  // public Command getAutonomousCommand() {
  // return Autos.exampleAuto(m_intakeSubsystem, m_indexerSubsystem,
  // m_feederSubsystem);
  // }
}
