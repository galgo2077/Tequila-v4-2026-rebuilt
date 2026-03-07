package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.*;

public class RobotContainer {
  // --- Subsistemas ---
  private final intakeSubsystem m_intake = new intakeSubsystem();
  private final IndexerSubsystem m_indexer = new IndexerSubsystem();
  private final feederSubsystem m_feeder = new feederSubsystem();
  private final fixedTurretSubsystem m_fixedTurret = new fixedTurretSubsystem();
  private final mobileTurretSubsystem m_mobileTurret = new mobileTurretSubsystem();
  private final climberSubsystem m_climber = new climberSubsystem();

  private final limelightTurret m_vision = new limelightTurret();
  private final mobileTurretSubsystem m_mobileTurret = new mobileTurretSubsystem(m_vision);
  private final fixedTurretSubsystem  m_fixedTurret  = new fixedTurretSubsystem(m_vision);

  // --- Controles ---
  private final CommandXboxController m_driverController = 
      new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = 
      new CommandXboxController(OIConstants.kMechanismControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    
    // --- INTAKE & EXTENSOR ---
    m_operatorController.leftTrigger().whileTrue(
        m_intake.runIntakeExtensorCommand(() -> true, () -> false, () -> false));

    m_operatorController.leftBumper().whileTrue(
        m_intake.runIntakeExtensorCommand(() -> false, () -> true, () -> false));
    m_operatorController.rightBumper+().whileTrue(
        m_intake.runIntakeExtensorCommand(() -> false, () -> false, () -> true));


    // --- INDEXER & ROLLER (Carga Manual) ---
    m_operatorController.x().whileTrue(
        m_indexer.runIndexerCommand(() -> true, () -> false, () -> false, () -> false));


    // =========================================================
    // SECUENCIA DE DISPARO DINÁMICA (RIGHT TRIGGER)
    // Sincroniza: Indexer carga + Shooter apunta con Mapa + Shooter móvil ON
    // =========================================================
    m_operatorController.rightTrigger().whileTrue(
        m_indexer.runIndexerCommand(() -> true, () -> false, () -> true, () -> false)
        .alongWith(m_fixedTurret.runFixedTurretCommand(() -> false, () -> false, true))
        .alongWith(m_mobileTurret.run(() -> {
            // Aquí puedes usar una distancia fija o Limelight
            double distance = 3.0; 
            m_mobileTurret.executeShotMap(distance);
        }))
    );


    // --- FEEDER (DISPARADOR FINAL) ---
    m_operatorController.rightBumper().whileTrue(
        m_feeder.runFeederCommand(() -> true));


    // --- CLIMBER ---
    m_climber.setDefaultCommand(
        m_climber.runClimberCommand(() -> -m_operatorController.getRightY()));


    // --- CONTROLES MANUALES (POV) ---
    m_operatorController.povUp().whileTrue(
        m_fixedTurret.runFixedTurretCommand(() -> true, () -> false, () -> false)
        .alongWith(m_mobileTurret.runManualCommand(() -> 0.5, () -> 0, () -> false))
    );

    m_operatorController.povDown().whileTrue(
        m_fixedTurret.runFixedTurretCommand(() -> false, () -> true, () -> false)
        .alongWith(m_mobileTurret.runManualCommand(() -> -0.5, () -> 0, () -> false))
    );
  }

  public Command getAutonomousCommand() {
    return null; 
  }
}