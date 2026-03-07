package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drive.JoystickSwerveCmd;
import frc.robot.subsystems.drive.DriveSubsystem;

import frc.robot.Constants.ModoRobot;          // ← era "Mode", ahora es "ModoRobot"
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.mechanism.*;
import frc.robot.subsystems.mechanism.io.*;


public class RobotContainer {

    // ── Subsystems ────────────────────────────────────────────────────────────


    private final feederSubsystem       m_feeder;
    private final intakeSubsystem       m_intake;
    private final IndexerSubsystem      m_indexer;
    private final mobileTurretSubsystem m_mobileTurret;
    private final fixedTurretSubsystem  m_fixedTurret;
    private final climberSubsystem      m_climber;

    // ── Controllers ───────────────────────────────────────────────────────────
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);

    // ── Auto Chooser ──────────────────────────────────────────────────────────
    SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {

        // ── Inyección de IO según modo ─────────────────────────────────────
        // Constants.currentMode se define en Constants.java como ModoRobot.REAL / SIM / REPLAY
        switch (Constants.currentMode) {

            case REAL -> {
                m_feeder       = new feederSubsystem(new FeederIOReal());
                m_intake       = new intakeSubsystem(new IntakeIOReal());
                m_indexer      = new IndexerSubsystem(new IndexerIOReal());
                m_mobileTurret = new mobileTurretSubsystem(new MobileTurretIOReal());
                m_fixedTurret  = new fixedTurretSubsystem(new FixedTurretIOReal());
                m_climber      = new climberSubsystem(new ClimberIOReal());
            }

            case SIM -> {
                m_feeder       = new feederSubsystem(new FeederIOSim());
                m_intake       = new intakeSubsystem(new IntakeIOSim());
                m_indexer      = new IndexerSubsystem(new IndexerIOSim());
                m_mobileTurret = new mobileTurretSubsystem(new MobileTurretIOSim());
                m_fixedTurret  = new fixedTurretSubsystem(new FixedTurretIOSim());
                m_climber      = new climberSubsystem(new ClimberIOSim());
            }

            default -> { // REPLAY — IO no hace nada, subsystem corre con logs
                m_feeder       = new feederSubsystem(new FeederIO() {});
                m_intake       = new intakeSubsystem(new IntakeIO() {});
                m_indexer      = new IndexerSubsystem(new IndexerIO() {});
                m_mobileTurret = new mobileTurretSubsystem(new MobileTurretIO() {});
                m_fixedTurret  = new fixedTurretSubsystem(new FixedTurretIO() {});
                m_climber      = new climberSubsystem(new ClimberIO() {});
            }
        }

        // ── PathPlanner ───────────────────────────────────────────────────
        Pathfinding.setPathfinder(new LocalADStar());
        PathfindingCommand.warmupCommand().schedule();

        // ── Comando por defecto del swerve ────────────────────────────────
        swerveSubsystem.setDefaultCommand(new JoystickSwerveCmd(
            swerveSubsystem,
            () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
            () ->  driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
            () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
            () ->  driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButton)
        ));

        configureBindings();

        // ── Auto chooser ──────────────────────────────────────────────────
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);
    }

    private void configureBindings() {
        new JoystickButton(driverJoystick, OIConstants.kDriverResetHeading)
            .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}