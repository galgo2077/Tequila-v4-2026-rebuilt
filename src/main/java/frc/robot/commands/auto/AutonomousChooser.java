package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.mechanisms.SuperstructureCommand;
// TODO: import frc.robot.subsystems.DriveSubsystem; (add when DriveSubsystem is created)

/**
 * AutonomousChooser — registro centralizado de Named Commands y
 * construcción del SendableChooser de modos autónomo.
 *
 * FLUJO DE USO EN RobotContainer:
 *
 *   // 1. Configurar AutoBuilder (una sola vez, antes de cualquier auto)
 *   AutonomousChooser.configureAutoBuilder(m_drive);
 *
 *   // 2. Crear el chooser (registra todos los named commands internamente)
 *   m_autoChooser = new AutonomousChooser(m_super);
 *
 *   // 3. En autonomousInit():
 *   m_autoChooser.getSelected().schedule();
 *
 * CONVENCIÓN DE NOMBRES DE ARCHIVOS:
 *   Los .auto de PathPlanner deben estar en:
 *   src/main/deploy/pathplanner/autos/
 *   Nombres exactos: "BomberCenter", "BomberSprint", "StrikerFeed",
 *                    "InterceptorBlitz", "TurretOnly", "JustLeave"
 */
public class AutonomousChooser {

    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    // ─────────────────────────────────────────────────────────────────────────
    // CONSTRUCTOR
    // ─────────────────────────────────────────────────────────────────────────
    public AutonomousChooser(SuperstructureCommand superstructure) {
        registerNamedCommands(superstructure);
        buildChooser(superstructure);
        SmartDashboard.putData("Auto Chooser", m_chooser);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // CONFIGURACIÓN DE AUTOBUILDER
    // Llamar UNA vez en RobotContainer antes de crear el AutonomousChooser
    // ─────────────────────────────────────────────────────────────────────────
    public static void configureAutoBuilder(DriveSubsystem drive) {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                drive::getPose,
                drive::resetOdometry,
                drive::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> drive.driveRobotRelative(speeds),
                new PPHolonomicDriveController(
                    AutoConstants.kPXController,       // kP translación X
                    AutoConstants.kPYController,       // kP translación Y
                    AutoConstants.kPThetaController    // kP rotación
                ),
                config,
                // Alliance flip automático: PathPlanner espeja paths para RED
                () -> DriverStation.getAlliance()
                      .map(a -> a == DriverStation.Alliance.Red)
                      .orElse(false),
                drive
            );
        } catch (Exception e) {
            System.err.println("[AutonomousChooser] Error cargando RobotConfig: " + e.getMessage());
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // REGISTRO DE NAMED COMMANDS
    // Cada string debe coincidir EXACTAMENTE con el nombre en PathPlanner GUI
    // ─────────────────────────────────────────────────────────────────────────
    private void registerNamedCommands(SuperstructureCommand superstructure) {

        // ── Intake ────────────────────────────────────────────────────────
        NamedCommands.registerCommand("IntakeDown",
            Commands.runOnce(() ->
                superstructure.requestState(SuperstructureCommand.State.INTAKING)));

        NamedCommands.registerCommand("IntakeUp",
            Commands.runOnce(() ->
                superstructure.requestState(SuperstructureCommand.State.IDLE)));

        // ── Disparo dual (Bomber) ─────────────────────────────────────────
        NamedCommands.registerCommand("PrepDual",
            Commands.runOnce(() ->
                superstructure.requestState(SuperstructureCommand.State.PREPPING_DUAL)));

        NamedCommands.registerCommand("ShootDual",
            buildShootCommand(superstructure,
                SuperstructureCommand.State.PREPPING_DUAL,
                SuperstructureCommand.State.SHOOTING_DUAL,
                () -> superstructure.isMobileShooterReady()
                   && superstructure.isFixedShooterReady()));

        // ── Disparo turret solo (Striker) ─────────────────────────────────
        NamedCommands.registerCommand("PrepTurret",
            Commands.runOnce(() ->
                superstructure.requestState(SuperstructureCommand.State.PREPPING_TURRET)));

        NamedCommands.registerCommand("ShootTurret",
            buildShootCommand(superstructure,
                SuperstructureCommand.State.PREPPING_TURRET,
                SuperstructureCommand.State.SHOOTING_TURRET,
                superstructure::isMobileShooterReady));

        // ── Feed a alianza ────────────────────────────────────────────────
        NamedCommands.registerCommand("FeedOut",
            Commands.runOnce(() ->
                superstructure.requestState(SuperstructureCommand.State.FEEDING_OUT)));

        // ── Detener todo ──────────────────────────────────────────────────
        NamedCommands.registerCommand("StopShoot",
            Commands.runOnce(() ->
                superstructure.requestState(SuperstructureCommand.State.IDLE)));
    }

    // ─────────────────────────────────────────────────────────────────────────
    // CONSTRUCCIÓN DEL CHOOSER
    // ─────────────────────────────────────────────────────────────────────────
    private void buildChooser(SuperstructureCommand superstructure) {

        // DEFAULT — emergencia
        m_chooser.setDefaultOption("JustLeave",
            new JustLeaveAuto(superstructure));

        // Opciones principales
        m_chooser.addOption("BomberCenter  (4 bolas)",
            new BomberCenterAuto(superstructure));

        m_chooser.addOption("BomberSprint  (2 bolas rapido)",
            new BomberSprintAuto(superstructure));

        m_chooser.addOption("TurretOnly    (3 bolas conservador)",
            new TurretOnlyAuto(superstructure));

        m_chooser.addOption("StrikerFeed   (feed alianza)",
            new StrikerFeedAuto(superstructure));

        m_chooser.addOption("InterceptorBlitz (defensa)",
            new InterceptorBlitzAuto(superstructure));
    }

    // ─────────────────────────────────────────────────────────────────────────
    // API
    // ─────────────────────────────────────────────────────────────────────────
    public Command getSelected() {
        return m_chooser.getSelected();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // HELPER INTERNO — comando de disparo con wait-for-ready
    // ─────────────────────────────────────────────────────────────────────────
    private static Command buildShootCommand(
            SuperstructureCommand superstructure,
            SuperstructureCommand.State prepState,
            SuperstructureCommand.State shootState,
            BooleanSupplierHelper readyCheck) {

        return Commands.sequence(
            Commands.runOnce(() -> superstructure.requestState(prepState)),
            Commands.waitUntil(readyCheck::get).withTimeout(2.0), // Máx 2s esperando
            Commands.runOnce(() -> superstructure.requestState(shootState)),
            Commands.waitUntil(() ->
                superstructure.getState() == SuperstructureCommand.State.IDLE)
                .withTimeout(1.5)
        );
    }

    @FunctionalInterface
    private interface BooleanSupplierHelper {
        boolean get();
    }
}
