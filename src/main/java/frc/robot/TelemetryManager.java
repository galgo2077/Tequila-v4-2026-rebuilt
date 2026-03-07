package frc.robot;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.mechanisms.SuperstructureCommand;
import frc.robot.subsystems.mechanism.IndexerSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.mechanism.fixedTurretSubsystem;
import frc.robot.subsystems.mechanism.mobileTurretSubsystem;

/**
 * TelemetryManager — publica todos los datos de Elastic Dashboard
 * definidos en la Sección 9 del spec de arquitectura.
 *
 * Extiende SubsystemBase para que periodic() corra automáticamente
 * cada 20ms. Los widgets de baja frecuencia (10Hz, 1Hz) usan contadores
 * de ciclo para no publicar en cada tick.
 *
 * WIRING EN RobotContainer:
 *   new TelemetryManager(m_drive, m_super, m_mTurret, m_fTurret, m_indexer);
 *   (basta con instanciar — periodic() se programa automáticamente)
 *
 * ELASTIC DASHBOARD:
 *   Importar los widgets en Elastic y conectarlos a los keys de SmartDashboard.
 *   Los keys están organizados por categoría para facilitar el layout.
 *
 * UPDATE RATES (según spec):
 *   50Hz  → cada ciclo    (20ms, counter no usado)
 *   20Hz  → cada 2 ciclos (40ms)
 *   10Hz  → cada 5 ciclos (100ms)
 *    1Hz  → cada 50 ciclos (1000ms)
 */
public class TelemetryManager extends SubsystemBase {

    // ── Subsistemas ───────────────────────────────────────────────────────────
    private final DriveSubsystem        m_drive;
    private final SuperstructureCommand m_super;
    private final mobileTurretSubsystem m_mTurret;
    private final fixedTurretSubsystem  m_fTurret;
    private final IndexerSubsystem      m_indexer;

    // ── Field widget (pose visual en Elastic) ─────────────────────────────────
    private final Field2d m_field = new Field2d();

    // ── Contador de ciclos ────────────────────────────────────────────────────
    private int m_cycleCount = 0;

    // ── Match timer (legacy, para el widget de 1Hz) ───────────────────────────
    private final Timer m_displayTimer = new Timer();

    // ─────────────────────────────────────────────────────────────────────────
    // CONSTRUCTOR
    // ─────────────────────────────────────────────────────────────────────────
    public TelemetryManager(DriveSubsystem drive,
                             SuperstructureCommand superstructure,
                             mobileTurretSubsystem mTurret,
                             fixedTurretSubsystem  fTurret,
                             IndexerSubsystem      indexer) {
        m_drive   = drive;
        m_super   = superstructure;
        m_mTurret = mTurret;
        m_fTurret = fTurret;
        m_indexer = indexer;

        // Registrar Field2d una vez — Elastic lo busca por key "Field"
        SmartDashboard.putData("Field", m_field);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // PERIODIC — 20ms
    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void periodic() {
        m_cycleCount++;

        publish50Hz();
        if (m_cycleCount % 2 == 0) publish20Hz();
        if (m_cycleCount % 5 == 0) publish10Hz();
        if (m_cycleCount % 50 == 0) {
            publish1Hz();
            m_cycleCount = 0; // Reset para evitar overflow
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 50Hz — pose, velocidades, turrets, ready indicators
    // ─────────────────────────────────────────────────────────────────────────
    private void publish50Hz() {

        // ── Field2d Pose (visualización 2D del robot en el campo) ─────────
        m_field.setRobotPose(m_drive.getPose());

        // ── Odometría X / Y ──────────────────────────────────────────────
        SmartDashboard.putNumber("Odometry/X (m)",  m_drive.getPose().getX());
        SmartDashboard.putNumber("Odometry/Y (m)",  m_drive.getPose().getY());
        SmartDashboard.putNumber("Odometry/Heading (deg)",
            m_drive.getPose().getRotation().getDegrees());

        // ── Flywheel RPM ──────────────────────────────────────────────────
        // TODO: exponer getShooterRPS() en mobileTurretSubsystem y fixedTurretSubsystem
        // SmartDashboard.putNumber("Shooter/Mobile RPM", m_mTurret.getShooterRPS() * 60.0);
        // SmartDashboard.putNumber("Shooter/Fixed RPM",  m_fTurret.getShooterRPS() * 60.0);

        // ── Turret Angle ──────────────────────────────────────────────────
        SmartDashboard.putNumber("Turret/Mobile Angle (revs)", m_mTurret.getAngleRevs());
        SmartDashboard.putNumber("Turret/Fixed Angle (revs)",  m_fTurret.getAngleRevs());

        // ── Shooter Ready Indicator ───────────────────────────────────────
        SmartDashboard.putBoolean("Shooter/Mobile Ready", m_super.isMobileShooterReady());
        SmartDashboard.putBoolean("Shooter/Fixed Ready",  m_super.isFixedShooterReady());
        SmartDashboard.putBoolean("Shooter/Both Ready",
            m_super.isMobileShooterReady() && m_super.isFixedShooterReady());
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 20Hz — superstructure state, indexer state, ball count
    // ─────────────────────────────────────────────────────────────────────────
    private void publish20Hz() {

        // ── Drive Mode Indicator ─────────────────────────────────────────
        SmartDashboard.putString("DriveMode/Active", m_super.getDriveMode().name());
        SmartDashboard.putBoolean("DriveMode/Is Bomber",
            m_super.getDriveMode() == SuperstructureCommand.DriveMode.BOMBER);
        SmartDashboard.putBoolean("DriveMode/Is Striker",
            m_super.getDriveMode() == SuperstructureCommand.DriveMode.STRIKER);
        SmartDashboard.putBoolean("DriveMode/Is Interceptor",
            m_super.getDriveMode() == SuperstructureCommand.DriveMode.INTERCEPTOR);

        // ── Superstructure State ──────────────────────────────────────────
        SmartDashboard.putString("Superstructure/State",       m_super.getState().name());
        SmartDashboard.putBoolean("Superstructure/ESTOP",
            m_super.getState() == SuperstructureCommand.State.ESTOP);

        // ── Indexer State (placeholder — exponer desde IndexerSubsystem) ──
        // SmartDashboard.putString("Indexer/State", m_indexer.getCurrentState().name());

        // ── Ball Count (placeholder — implementar sensor lógica) ──────────
        // SmartDashboard.putNumber("Superstructure/Ball Count", m_super.getBallCount());
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 10Hz — voltaje batería, CAN utilization
    // ─────────────────────────────────────────────────────────────────────────
    private void publish10Hz() {

        // ── Battery Voltage ───────────────────────────────────────────────
        double batteryVolts = RobotController.getBatteryVoltage();
        SmartDashboard.putNumber("System/Battery Voltage", batteryVolts);
        SmartDashboard.putBoolean("System/Battery Low", batteryVolts < 11.5);

        // ── CAN Bus Utilization ───────────────────────────────────────────
        CANStatus canStatus = RobotController.getCANStatus();
        SmartDashboard.putNumber("System/CAN Utilization (%)",
            canStatus.percentBusUtilization * 100.0);
        SmartDashboard.putNumber("System/CAN TX Error",  canStatus.txFullCount);
        SmartDashboard.putNumber("System/CAN RX Error",  canStatus.receiveErrorCount);

        // ── Robot Enable State ────────────────────────────────────────────
        SmartDashboard.putBoolean("System/Robot Enabled", DriverStation.isEnabled());
        SmartDashboard.putBoolean("System/Is Autonomous", DriverStation.isAutonomous());
        SmartDashboard.putBoolean("System/Is Teleop",     DriverStation.isTeleop());
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 1Hz — match timer
    // ─────────────────────────────────────────────────────────────────────────
    private void publish1Hz() {

        // ── Match Timer ───────────────────────────────────────────────────
        double matchTime = DriverStation.getMatchTime();
        SmartDashboard.putNumber("System/Match Time (s)", matchTime);
        SmartDashboard.putBoolean("System/Endgame",       matchTime > 0 && matchTime <= 30.0);
        SmartDashboard.putBoolean("System/Final 10s",     matchTime > 0 && matchTime <= 10.0);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // ELASTIC DASHBOARD KEY MAP
    // ─────────────────────────────────────────────────────────────────────────
    //
    // Copiar estos keys exactamente en los widgets de Elastic:
    //
    // CAMPO / POSE
    //   "Field"                        → Field2d widget (pose visual)
    //   "Odometry/X (m)"               → Number Display
    //   "Odometry/Y (m)"               → Number Display
    //   "Odometry/Heading (deg)"       → Number Display / Gyro widget
    //
    // DRIVE MODE
    //   "DriveMode/Active"             → Text Display (B / S / I)
    //   "DriveMode/Is Bomber"          → Boolean Box (verde)
    //   "DriveMode/Is Striker"         → Boolean Box (amarillo)
    //   "DriveMode/Is Interceptor"     → Boolean Box (rojo)
    //
    // SHOOTERS
    //   "Shooter/Mobile RPM"           → Number Display / Gauge
    //   "Shooter/Fixed RPM"            → Number Display / Gauge
    //   "Shooter/Mobile Ready"         → Boolean Box
    //   "Shooter/Fixed Ready"          → Boolean Box
    //   "Shooter/Both Ready"           → Large Boolean Indicator
    //
    // TURRET
    //   "Turret/Mobile Angle (revs)"   → Number Display
    //   "Turret/Fixed Angle (revs)"    → Number Display
    //
    // SUPERSTRUCTURE
    //   "Superstructure/State"         → Text Display
    //   "Superstructure/ESTOP"         → Boolean Box (rojo = activo)
    //
    // SISTEMA
    //   "System/Battery Voltage"       → Voltage Gauge
    //   "System/Battery Low"           → Boolean Box (rojo = bajo)
    //   "System/CAN Utilization (%)"   → Number Display / Bar
    //   "System/Match Time (s)"        → Number Display / Countdown
    //   "System/Endgame"               → Boolean Box (naranja)
    //   "System/Final 10s"             → Boolean Box (rojo parpadeante)
}
