package frc.robot.commands.mechanisms;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveModeConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.FixedTurretConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.MobileTurretConstants;
import frc.robot.subsystems.mechanism.IndexerSubsystem;
import frc.robot.subsystems.mechanism.climberSubsystem;
import frc.robot.subsystems.mechanism.feederSubsystem;
import frc.robot.subsystems.mechanism.fixedTurretSubsystem;
import frc.robot.subsystems.mechanism.intakeSubsystem;
import frc.robot.subsystems.vision.limelightTurret;
import frc.robot.subsystems.mechanism.mobileTurretSubsystem;

import java.util.function.DoubleSupplier;

/**
 * SuperstructureCommand — coordinador central de todos los mecanismos.
 *
 * PATRÓN:
 *   Un Command perpetuo (isFinished = false) que corre todo el match.
 *   addRequirements() reclama ownership de TODOS los subsistemas de hardware,
 *   lo que garantiza que ningún otro comando pueda tocarlos mientras este corre.
 *
 * TRANSICIONES:
 *   Los botones llaman requestState() via Commands.runOnce(() -> m_super.requestState(...)).
 *   Ese runOnce NO requiere ningún subsistema → no interrumpe este comando.
 *   requestState() evalúa la guardia y, si pasa, actualiza m_currentState.
 *   El próximo ciclo de execute() ya aplica el comportamiento del nuevo estado.
 *
 * REGLA DE ORO:
 *   execute() SOLO llama métodos set*Voltage() y stop() de los subsistemas.
 *   Nunca se programan sub-comandos desde dentro de execute().
 *
 * PROGRAMACIÓN:
 *   Programar UNA vez al inicio del match.
 *   No cancelar nunca — si se interrumpe (ej. ESTOP de DS), end() detiene todo.
 */
public class SuperstructureCommand extends Command {

    // ─────────────────────────────────────────────────────────────────────────
    // ESTADOS
    // ─────────────────────────────────────────────────────────────────────────
    public enum State {
        IDLE,
        INTAKING,
        PREPPING_DUAL,
        SHOOTING_DUAL,
        PREPPING_TURRET,
        SHOOTING_TURRET,
        FEEDING_OUT,
        /** Reversa de intake e indexer para expulsar piezas. */
        OUTTAKE,
        CLIMBING,
        ESTOP,
    }

    // ─────────────────────────────────────────────────────────────────────────
    // MODO DE CONDUCCIÓN
    // ─────────────────────────────────────────────────────────────────────────
    public enum DriveMode { BOMBER, STRIKER, INTERCEPTOR }

    // ─────────────────────────────────────────────────────────────────────────
    // SUBSISTEMAS
    // ─────────────────────────────────────────────────────────────────────────
    private final intakeSubsystem       m_intake;
    private final IndexerSubsystem      m_indexer;
    private final feederSubsystem       m_feeder;
    private final mobileTurretSubsystem m_mobileTurret;
    private final fixedTurretSubsystem  m_fixedTurret;
    private final climberSubsystem      m_climber;
    private final limelightTurret       m_vision;

    // ─────────────────────────────────────────────────────────────────────────
    // INPUT EXTERNO — Joystick del climber
    // Inyectado en el constructor desde RobotContainer
    // ─────────────────────────────────────────────────────────────────────────
    private final DoubleSupplier m_climberAxis;

    // ─────────────────────────────────────────────────────────────────────────
    // ESTADO INTERNO
    // ─────────────────────────────────────────────────────────────────────────
    private State     m_currentState        = State.IDLE;
    private DriveMode m_driveMode           = DriveMode.BOMBER;
    private boolean   m_estopAcknowledged   = false;

    // Timer para los estados de disparo — evita withTimeout() con sub-comandos
    private final Timer m_shootTimer        = new Timer();
    private boolean     m_shootTimerRunning = false;

    // ─────────────────────────────────────────────────────────────────────────
    // CONSTRUCTOR
    // ─────────────────────────────────────────────────────────────────────────
    public SuperstructureCommand(
            intakeSubsystem intake,
            IndexerSubsystem indexer,
            feederSubsystem feeder,
            mobileTurretSubsystem mobileTurret,
            fixedTurretSubsystem fixedTurret,
            climberSubsystem climber,
            limelightTurret vision,
            DoubleSupplier climberAxis) {

        m_intake       = intake;
        m_indexer      = indexer;
        m_feeder       = feeder;
        m_mobileTurret = mobileTurret;
        m_fixedTurret  = fixedTurret;
        m_climber      = climber;
        m_vision       = vision;
        m_climberAxis  = climberAxis;

        // Reclamar ownership de TODOS los subsistemas de hardware.
        // limelightTurret no tiene motores → no se incluye.
        addRequirements(intake, indexer, feeder, mobileTurret, fixedTurret, climber);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // API PÚBLICA — llamada desde botones via Commands.runOnce(...)
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Solicita una transición de estado.
     * Si el guard falla, el estado NO cambia.
     *
     * Uso en RobotContainer:
     *   new JoystickButton(ctrl, 1)
     *       .onTrue(Commands.runOnce(() -> m_super.requestState(State.INTAKING)));
     */
    public void requestState(State next) {
        if (next == State.ESTOP) {
            enterState(next);
            return;
        }
        if (m_currentState == State.ESTOP && !m_estopAcknowledged) {
            System.out.println("[Superstructure] ESTOP activo — llama clearEstop() primero");
            return;
        }
        if (!guardFor(next)) {
            System.out.println("[Superstructure] Guard falló: " + m_currentState + " → " + next);
            return;
        }
        enterState(next);
    }

    /** Reconoce el ESTOP para permitir transiciones nuevamente. */
    public void clearEstop() {
        if (m_currentState == State.ESTOP) {
            m_estopAcknowledged = true;
            enterState(State.IDLE);
            System.out.println("[Superstructure] ESTOP reconocido → IDLE");
        }
    }

    /** Cambia el modo de conducción desde RobotContainer. */
    public void setDriveMode(DriveMode mode) {
        m_driveMode = mode;
        System.out.println("[Superstructure] DriveMode: " + mode);
    }

    public State     getState()     { return m_currentState; }
    public DriveMode getDriveMode() { return m_driveMode;    }

    // ─────────────────────────────────────────────────────────────────────────
    // COMMAND — CICLO DE VIDA
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        enterState(State.IDLE);
        System.out.println("[Superstructure] Iniciado.");
    }

    /**
     * Corre cada 20 ms.
     * Lee m_currentState y aplica el comportamiento correspondiente.
     * SOLO llama métodos de voltaje — nunca programa sub-comandos.
     */
    @Override
    public void execute() {
        switch (m_currentState) {

            case IDLE        -> executeIdle();
            case INTAKING    -> executeIntaking();
            case PREPPING_DUAL    -> executePrepDual();
            case SHOOTING_DUAL    -> executeShootDual();
            case PREPPING_TURRET  -> executePrepTurret();
            case SHOOTING_TURRET  -> executeShootTurret();
            case FEEDING_OUT -> executeFeedingOut();
            case OUTTAKE     -> executeOuttake();
            case CLIMBING    -> executeClimbing();
            case ESTOP       -> executeEstop();
        }

        publishDashboard();
    }

    /** Nunca termina — corre todo el match. */
    @Override
    public boolean isFinished() {
        return false;
    }

    /** Llamado si el DS deshabilita el robot o el comando es interrumpido. */
    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
        m_indexer.stop();
        m_feeder.stop();
        m_mobileTurret.stop();
        m_fixedTurret.stop();
        m_climber.stop();
        System.out.println("[Superstructure] Terminado (interrupted=" + interrupted + ")");
    }

    // ─────────────────────────────────────────────────────────────────────────
    // COMPORTAMIENTOS POR ESTADO
    // ─────────────────────────────────────────────────────────────────────────

    // IDLE — todo quieto, turrets rastrean target si hay visión
    private void executeIdle() {
        m_intake.setIntakeVoltage(0);
        m_intake.setExtensorVoltage(0);
        m_indexer.setRollerVoltage(0);
        m_indexer.setIndexerVoltage(0, 0);
        m_feeder.setFeederVoltage(0);
        m_climber.setClimberVoltage(0);

        // Turret móvil: rastrear TX si hay target; si no, mantener posición
        if (m_vision.hasTarget() && m_mobileTurret.isFullyHomed()) {
            m_mobileTurret.setRotationVoltage(
                m_vision.getRotationVoltage(MobileTurretConstants.kRotationSpeed * 12.0));
            m_mobileTurret.setAngleVoltage(
                m_vision.getHoodVoltage(
                    m_mobileTurret.getAngleRevs(),
                    MobileTurretConstants.kHoodGearRatio,
                    MobileTurretConstants.kAngleSpeed * 12.0,
                    m_mobileTurret.isAngleHomed()));
        } else {
            m_mobileTurret.setRotationVoltage(0);
            m_mobileTurret.setAngleVoltage(0);
        }
        m_mobileTurret.setShooterVoltage(0);
        m_fixedTurret.setAngleVoltage(0);
        m_fixedTurret.setShooterVoltage(0);
    }

    // INTAKING — intake extendido, rollers, indexer hacia ambos shooters
    private void executeIntaking() {
        // Extensor: mover hacia deployed si aún no llegó
        double extError = IntakeConstants.kExtensorDeployed - m_intake.getExtensorPositionRevs();
        m_intake.setExtensorVoltage(m_intake.isHomed()
            ? clamp(extError * 0.4, -6.0, 6.0) : 0.0);

        m_intake.setIntakeVoltage(IntakeConstants.kIntakeSpeed * 12.0);
        m_indexer.setRollerVoltage(IndexerConstants.kRollerSpeed * 12.0);
        m_indexer.setIndexerVoltage(
            IndexerConstants.kIndexerSpeed * 12.0,
            IndexerConstants.kIndexerSpeed * 12.0);
        m_feeder.setFeederVoltage(0);
        m_climber.setClimberVoltage(0);

        // Turret sigue rastreando en background
        if (m_vision.hasTarget() && m_mobileTurret.isFullyHomed()) {
            m_mobileTurret.setRotationVoltage(
                m_vision.getRotationVoltage(MobileTurretConstants.kRotationSpeed * 12.0));
        } else {
            m_mobileTurret.setRotationVoltage(0);
        }
        m_mobileTurret.setAngleVoltage(0);
        m_mobileTurret.setShooterVoltage(0);
        m_fixedTurret.setAngleVoltage(0);
        m_fixedTurret.setShooterVoltage(0);
    }

    // PREPPING_DUAL — ambos flywheels ramping, hoods apuntando, feeder bloqueado
    private void executePrepDual() {
        m_intake.setIntakeVoltage(0);
        m_intake.setExtensorVoltage(retractExtensor());
        m_indexer.setRollerVoltage(0);
        m_indexer.setIndexerVoltage(
            IndexerConstants.kIndexerSpeed * 12.0,
            IndexerConstants.kIndexerSpeed * 12.0);
        m_feeder.setFeederVoltage(0);
        m_climber.setClimberVoltage(0);

        applyMobileTurretAim();
        applyFixedTurretAim();
    }

    // SHOOTING_DUAL — feeder activo; timer auto-retorna a IDLE
    private void executeShootDual() {
        m_intake.setIntakeVoltage(0);
        m_intake.setExtensorVoltage(retractExtensor());
        m_indexer.setRollerVoltage(0);
        m_indexer.setIndexerVoltage(
            IndexerConstants.kIndexerSpeed * 12.0,
            IndexerConstants.kIndexerSpeed * 12.0);
        m_feeder.setFeederVoltage(FeederConstants.kFeederSpeed * 12.0);
        m_climber.setClimberVoltage(0);

        applyMobileTurretAim();
        applyFixedTurretAim();

        if (m_shootTimer.hasElapsed(FeederConstants.kShootDelaySeconds)) {
            m_shootTimerRunning = false;
            enterState(State.IDLE);
        }
    }

    // PREPPING_TURRET — solo turret móvil, fixed apagado
    private void executePrepTurret() {
        m_intake.setIntakeVoltage(0);
        m_intake.setExtensorVoltage(retractExtensor());
        m_indexer.setRollerVoltage(0);
        m_indexer.setIndexerVoltage(
            IndexerConstants.kIndexerSpeed * 12.0,
            0.0); // Solo lado izquierdo hacia turret móvil
        m_feeder.setFeederVoltage(0);
        m_climber.setClimberVoltage(0);

        applyMobileTurretAim();
        m_fixedTurret.setAngleVoltage(0);
        m_fixedTurret.setShooterVoltage(0);
    }

    // SHOOTING_TURRET — solo turret móvil con feeder
    private void executeShootTurret() {
        m_intake.setIntakeVoltage(0);
        m_intake.setExtensorVoltage(retractExtensor());
        m_indexer.setRollerVoltage(0);
        m_indexer.setIndexerVoltage(
            IndexerConstants.kIndexerSpeed * 12.0,
            0.0);
        m_feeder.setFeederVoltage(FeederConstants.kFeederSpeed * 12.0);
        m_climber.setClimberVoltage(0);

        applyMobileTurretAim();
        m_fixedTurret.setAngleVoltage(0);
        m_fixedTurret.setShooterVoltage(0);

        if (m_shootTimer.hasElapsed(FeederConstants.kShootDelaySeconds)) {
            m_shootTimerRunning = false;
            enterState(State.IDLE);
        }
    }

    // OUTTAKE — reversa de intake e indexer para expulsar piezas
    private void executeOuttake() {
        double extError = 0.0 - m_intake.getExtensorPositionRevs();
        m_intake.setExtensorVoltage(m_intake.isHomed() ? clamp(extError * 0.4, -6.0, 6.0) : 0.0);
        m_intake.setIntakeVoltage(-IntakeConstants.kIntakeSpeed * 12.0); // Reversa
        m_indexer.setRollerVoltage(-IndexerConstants.kRollerSpeed * 12.0);
        m_indexer.setIndexerVoltage(
            -IndexerConstants.kIndexerSpeed * 12.0,
            -IndexerConstants.kIndexerSpeed * 12.0);
        m_feeder.setFeederVoltage(-FeederConstants.kFeederSpeed * 12.0);
        m_mobileTurret.setRotationVoltage(0);
        m_mobileTurret.setAngleVoltage(0);
        m_mobileTurret.setShooterVoltage(0);
        m_fixedTurret.setAngleVoltage(0);
        m_fixedTurret.setShooterVoltage(0);
        m_climber.setClimberVoltage(0);
    }

    // FEEDING_OUT — pasar pieza a aliado con velocidad media
    private void executeFeedingOut() {
        m_intake.setIntakeVoltage(0);
        m_intake.setExtensorVoltage(retractExtensor());
        m_indexer.setRollerVoltage(0);
        m_indexer.setIndexerVoltage(
            IndexerConstants.kIndexerSpeed * 12.0,
            0.0);
        m_feeder.setFeederVoltage(FeederConstants.kFeederSpeed * 0.5 * 12.0); // Velocidad media
        m_climber.setClimberVoltage(0);

        // Turret rota a ángulo fijo de alianza (sin Limelight)
        // TODO: Implementar rotación a ángulo de alianza cuando se defina ese ángulo
        m_mobileTurret.setRotationVoltage(0);
        m_mobileTurret.setAngleVoltage(0);
        m_mobileTurret.setShooterVoltage(MobileTurretConstants.kShooterSpeed * 12.0);
        m_fixedTurret.setAngleVoltage(0);
        m_fixedTurret.setShooterVoltage(0);
    }

    // CLIMBING — joystick directo al climber, todo lo demás quieto
    private void executeClimbing() {
        m_intake.setIntakeVoltage(0);
        m_intake.setExtensorVoltage(retractExtensor());
        m_indexer.setRollerVoltage(0);
        m_indexer.setIndexerVoltage(0, 0);
        m_feeder.setFeederVoltage(0);
        m_mobileTurret.setRotationVoltage(0);
        m_mobileTurret.setAngleVoltage(0);
        m_mobileTurret.setShooterVoltage(0);
        m_fixedTurret.setAngleVoltage(0);
        m_fixedTurret.setShooterVoltage(0);

        if (!m_climber.isHomed()) {
            m_climber.setClimberVoltage(0);
            return;
        }
        double input = m_climberAxis.getAsDouble();
        m_climber.setClimberVoltage(
            Math.abs(input) > 0.1 ? input * 12.0 : 0.0);
    }

    // ESTOP — todo a 0 cada ciclo (no solo en la transición)
    private void executeEstop() {
        m_intake.stop();
        m_indexer.stop();
        m_feeder.stop();
        m_mobileTurret.stop();
        m_fixedTurret.stop();
        m_climber.stop();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // GUARDIAS DE TRANSICIÓN
    // ─────────────────────────────────────────────────────────────────────────
    private boolean guardFor(State next) {
        return switch (next) {
            case IDLE            -> true;
            case INTAKING        -> true;
            case PREPPING_DUAL   -> m_mobileTurret.isFullyHomed()
                                    && m_fixedTurret.isHomed()
                                    && isDualAllowed();
            case SHOOTING_DUAL   -> m_currentState == State.PREPPING_DUAL
                                    && isMobileShooterReady()
                                    && isFixedShooterReady()
                                    && isDualAllowed();
            case PREPPING_TURRET -> m_mobileTurret.isFullyHomed();
            case SHOOTING_TURRET -> m_currentState == State.PREPPING_TURRET
                                    && isMobileShooterReady();
            case FEEDING_OUT     -> m_mobileTurret.isFullyHomed();
            case OUTTAKE         -> true;
            case CLIMBING        -> m_driveMode == DriveMode.BOMBER
                                    && m_climber.isHomed()
                                    && DriveModeConstants.kBomberClimberEnabled;
            case ESTOP           -> true;
        };
    }

    // ─────────────────────────────────────────────────────────────────────────
    // LÓGICA DE ENTRAR A UN ESTADO
    // ─────────────────────────────────────────────────────────────────────────
    private void enterState(State next) {
        System.out.println("[Superstructure] " + m_currentState + " → " + next);
        m_currentState = next;

        // Iniciar timer si entramos a un estado de disparo
        if (next == State.SHOOTING_DUAL || next == State.SHOOTING_TURRET) {
            m_shootTimer.reset();
            m_shootTimer.start();
            m_shootTimerRunning = true;
        } else if (m_shootTimerRunning) {
            m_shootTimer.stop();
            m_shootTimerRunning = false;
        }

        if (next != State.ESTOP) {
            m_estopAcknowledged = false;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // HELPERS DE COMPORTAMIENTO
    // ─────────────────────────────────────────────────────────────────────────

    /** Aplica aim + ramp al turret móvil. */
    private void applyMobileTurretAim() {
        if (!m_mobileTurret.isFullyHomed()) {
            m_mobileTurret.setRotationVoltage(0);
            m_mobileTurret.setAngleVoltage(0);
            m_mobileTurret.setShooterVoltage(0);
            return;
        }
        if (m_vision.hasTarget()) {
            m_mobileTurret.setRotationVoltage(
                m_vision.getRotationVoltage(MobileTurretConstants.kRotationSpeed * 12.0));
            m_mobileTurret.setAngleVoltage(
                m_vision.getHoodVoltage(
                    m_mobileTurret.getAngleRevs(),
                    MobileTurretConstants.kHoodGearRatio,
                    MobileTurretConstants.kAngleSpeed * 12.0,
                    m_mobileTurret.isAngleHomed()));
        } else {
            m_mobileTurret.setRotationVoltage(0);
            m_mobileTurret.setAngleVoltage(0);
        }
        m_mobileTurret.setShooterVoltage(MobileTurretConstants.kShooterSpeed * 12.0);
    }

    /** Aplica aim + ramp al shooter fijo. */
    private void applyFixedTurretAim() {
        if (!m_fixedTurret.isHomed() || !isDualAllowed()) {
            m_fixedTurret.setAngleVoltage(0);
            m_fixedTurret.setShooterVoltage(0);
            return;
        }
        if (m_vision.hasTarget()) {
            m_fixedTurret.setAngleVoltage(
                m_vision.getHoodVoltage(
                    m_fixedTurret.getAngleRevs(),
                    FixedTurretConstants.kGearRatio,
                    FixedTurretConstants.kAngleSpeed * 12.0,
                    m_fixedTurret.isHomed()));
        } else {
            m_fixedTurret.setAngleVoltage(0);
        }
        m_fixedTurret.setShooterVoltage(FixedTurretConstants.kShooterSpeed * 12.0);
    }

    /**
     * Voltaje de retracción proporcional para el extensor.
     * Devuelve 0 si ya está en posición o si no está homeado.
     */
    private double retractExtensor() {
        if (!m_intake.isHomed()) return 0.0;
        double error = 0.0 - m_intake.getExtensorPositionRevs(); // target = retracted = 0
        return Math.abs(error) < 0.3 ? 0.0 : clamp(error * 0.4, -6.0, 6.0);
    }

    /** ¿El turret móvil está listo para disparar? Hood en ángulo ±1 rev. */
    public boolean isMobileShooterReady() {
        if (!m_mobileTurret.isFullyHomed() || !m_vision.hasTarget()) return false;
        double currentRevs = m_mobileTurret.getAngleRevs();
        double idealRevs   = m_vision.getIdealHoodRevs(MobileTurretConstants.kHoodGearRatio);
        return Math.abs(currentRevs - idealRevs) < 1.0;
        // TODO: agregar isFlywheelAtSpeed() cuando el PID de velocidad esté implementado
    }

    /** ¿El shooter fijo está listo para disparar? */
    public boolean isFixedShooterReady() {
        if (!m_fixedTurret.isHomed() || !m_vision.hasTarget()) return false;
        double currentRevs = m_fixedTurret.getAngleRevs();
        double idealRevs   = m_vision.getIdealHoodRevs(FixedTurretConstants.kGearRatio);
        return Math.abs(currentRevs - idealRevs) < 1.0;
        // TODO: agregar isFlywheelAtSpeed() cuando el PID de velocidad esté implementado
    }

    private boolean isDualAllowed() {
        return switch (m_driveMode) {
            case BOMBER      -> DriveModeConstants.kBomberFixedEnabled   && DriveModeConstants.kBomberTurretEnabled;
            case STRIKER     -> DriveModeConstants.kStrikerFixedEnabled  && DriveModeConstants.kStrikerTurretEnabled;
            case INTERCEPTOR -> false;
        };
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // ─────────────────────────────────────────────────────────────────────────
    // DASHBOARD
    // ─────────────────────────────────────────────────────────────────────────
    private void publishDashboard() {
        SmartDashboard.putString ("Superstructure/State",          m_currentState.name());
        SmartDashboard.putString ("Superstructure/Drive Mode",     m_driveMode.name());
        SmartDashboard.putBoolean("Superstructure/ESTOP Active",   m_currentState == State.ESTOP);
        SmartDashboard.putBoolean("Superstructure/Dual Allowed",   isDualAllowed());
        SmartDashboard.putBoolean("Superstructure/Mobile Ready",   isMobileShooterReady());
        SmartDashboard.putBoolean("Superstructure/Fixed Ready",    isFixedShooterReady());
        SmartDashboard.putBoolean("Superstructure/Has Target",     m_vision.hasTarget());
        SmartDashboard.putNumber ("Superstructure/Shoot Timer",    m_shootTimer.get());
    }
}
