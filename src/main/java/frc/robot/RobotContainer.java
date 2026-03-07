package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.DriveModeConstants;
import frc.robot.commands.drive.*;
import frc.robot.commands.mechanisms.*;
import frc.robot.subsystems.mechanism.*;
import frc.robot.subsystems.vision.limelightTurret;

public class RobotContainer {

    // ── Controladores ────────────────────────────────────────────────────────
    private final CommandXboxController m_driverCtrl =
        new CommandXboxController(OIConstants.kDriverControllerPort);
    private final CommandXboxController m_mechCtrl =
        new CommandXboxController(OIConstants.kMechanismsControllerPort);

    // ── Subsistemas ───────────────────────────────────────────────────────────
    private final limelightTurret       m_vision       = new limelightTurret();
    private final intakeSubsystem       m_intake       = new intakeSubsystem();
    private final IndexerSubsystem      m_indexer      = new IndexerSubsystem();
    private final feederSubsystem       m_feeder       = new feederSubsystem();
    private final mobileTurretSubsystem m_mobileTurret = new mobileTurretSubsystem(m_vision);
    private final fixedTurretSubsystem  m_fixedTurret  = new fixedTurretSubsystem(m_vision);
    private final climberSubsystem      m_climber      = new climberSubsystem();

    // ── Superstructure ────────────────────────────────────────────────────────
    private final SuperstructureCommand m_super = new SuperstructureCommand(
        m_intake, m_indexer, m_feeder, m_mobileTurret, m_fixedTurret, m_climber, m_vision,
        () -> m_mechCtrl.getRawAxis(1)
    );

    // ─────────────────────────────────────────────────────────────────────────
    public RobotContainer() {
        // Programar SuperstructureCommand perpetuo
        m_super.schedule();

        // Homing al inicio (todos en paralelo)
        Commands.parallel(
            m_intake.homingCommand(),
            m_mobileTurret.homeAllCommand(),
            m_fixedTurret.homingCommand(),
            m_climber.homingCommand()
        ).schedule();

        configureDriverButtons();
        configureMechButtons();
    }

    // ── Botones DRIVER ────────────────────────────────────────────────────────
    private void configureDriverButtons() {
        // Orbit target — LB (solo Bomber)
        m_driverCtrl.leftBumper().whileTrue(new OrbitTargetCmd(
            null, // TODO: agregar DriveSubsystem
            () -> -m_driverCtrl.getRawAxis(OIConstants.kDriverYAxis),
            () -> -m_driverCtrl.getRawAxis(OIConstants.kDriverXAxis),
            isBlueAlliance()
        ));

        // Zero heading
        // m_driverCtrl.start().onTrue(new ZeroHeadingCmd(m_drive));

        // D-Pad snap a ángulos (requiere DriveSubsystem)
        // m_driverCtrl.povUp()   .whileTrue(new SnapToAngleCmd(m_drive, xSup, ySup,   0));
        // m_driverCtrl.povRight().whileTrue(new SnapToAngleCmd(m_drive, xSup, ySup,  90));
        // m_driverCtrl.povDown() .whileTrue(new SnapToAngleCmd(m_drive, xSup, ySup, 180));
        // m_driverCtrl.povLeft() .whileTrue(new SnapToAngleCmd(m_drive, xSup, ySup, 270));

        // Drive modes
        m_driverCtrl.back().onTrue(
            new SetDriveModeCmd(m_super, SuperstructureCommand.DriveMode.BOMBER));
        m_driverCtrl.start().onTrue(
            new SetDriveModeCmd(m_super, SuperstructureCommand.DriveMode.STRIKER));
    }

    // ── Botones MECANISMOS ────────────────────────────────────────────────────
    private void configureMechButtons() {

        // RT — Intake
        m_mechCtrl.rightTrigger(0.3)
            .onTrue(new IntakeCmd(m_super))
            .onFalse(Commands.runOnce(() ->
                m_super.requestState(SuperstructureCommand.State.IDLE)));

        // LT — Outtake
        m_mechCtrl.leftTrigger(0.3)
            .onTrue(new OuttakeCmd(m_super))
            .onFalse(Commands.runOnce(() ->
                m_super.requestState(SuperstructureCommand.State.IDLE)));

        // RB — Shoot: dual en Bomber, turret solo en Striker/Interceptor
        m_mechCtrl.rightBumper().onTrue(Commands.either(
            new ShootDualCmd(m_super),
            new ShootTurretCmd(m_super),
            () -> m_super.getDriveMode() == SuperstructureCommand.DriveMode.BOMBER
        ));

        // B — Feed out
        m_mechCtrl.b()
            .onTrue(new FeedOutCmd(m_super))
            .onFalse(Commands.runOnce(() ->
                m_super.requestState(SuperstructureCommand.State.IDLE)));

        // X — Pre-calentar shooters
        m_mechCtrl.x()
            .onTrue(new PrepShootersCmd(m_super))
            .onFalse(Commands.runOnce(() ->
                m_super.requestState(SuperstructureCommand.State.IDLE)));

        // Y — Climber
        m_mechCtrl.y()
            .onTrue(new ClimbCmd(m_super))
            .onFalse(new ClimbReleaseCmd(m_super));

        // Back — ESTOP
        m_mechCtrl.back().onTrue(
            Commands.runOnce(() ->
                m_super.requestState(SuperstructureCommand.State.ESTOP)));

        // LB — Reconocer ESTOP
        m_mechCtrl.leftBumper().onTrue(
            Commands.runOnce(m_super::clearEstop));

        // Drive modes desde mechCtrl
        m_mechCtrl.povLeft().onTrue(
            new SetDriveModeCmd(m_super, SuperstructureCommand.DriveMode.BOMBER));
        m_mechCtrl.povRight().onTrue(
            new SetDriveModeCmd(m_super, SuperstructureCommand.DriveMode.STRIKER));
        m_mechCtrl.povDown().onTrue(
            new SetDriveModeCmd(m_super, SuperstructureCommand.DriveMode.INTERCEPTOR));
    }

    // ── Auto ─────────────────────────────────────────────────────────────────
    public Command getAutonomousCommand() {
        // TODO: conectar AutonomousChooser cuando DriveSubsystem esté implementado
        return Commands.none();
    }

    // ── Helpers ──────────────────────────────────────────────────────────────
    private boolean isBlueAlliance() {
        return DriverStation.getAlliance()
            .map(a -> a == DriverStation.Alliance.Blue)
            .orElse(true);
    }
}
