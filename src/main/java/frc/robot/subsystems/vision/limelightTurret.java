package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShootingConfigs;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

/**
 * limelightTurret — Subsistema centralizado de visión para torretas.
 *
 * Lee el Limelight UNA sola vez por ciclo (periodic) y expone los
 * resultados calculados a mobileTurretSubsystem y fixedTurretSubsystem.
 *
 * USO EN RobotContainer:
 *   limelightTurret m_vision = new limelightTurret();
 *   mobileTurretSubsystem m_mobileTurret = new mobileTurretSubsystem(m_vision);
 *   fixedTurretSubsystem  m_fixedTurret  = new fixedTurretSubsystem(m_vision);
 *
 * Cada subsistema de torreta debe:
 *   1. Recibir un limelightTurret en su constructor.
 *   2. Eliminar su bloque de lectura Limelight de periodic().
 *   3. Reemplazar sus métodos de visión por las llamadas de esta clase.
 *
 * TODO (antes de regional):
 *   - Medir kCameraHeightMeters con el robot en suelo plano.
 *   - Mover kCameraHeightMeters a VisionConstants en Constants.java.
 *   - Agregar más puntos al ShotMap en ShootingConfigs.
 */
public class limelightTurret extends SubsystemBase {

    // ─────────────────────────────────────────────────────────────────────────
    // CONSTANTES DE MONTAJE
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Altura del lente de la cámara sobre el suelo (metros).
     * MEDIR en el robot real. Valor provisional hasta medición.
     * Mover a VisionConstants cuando esté confirmado.
     */
    private static final double kCameraHeightMeters = 0.50; // TODO: medir

    // ─────────────────────────────────────────────────────────────────────────
    // ESTADO CACHEADO — actualizado en periodic()
    // ─────────────────────────────────────────────────────────────────────────
    private boolean m_hasTarget      = false;
    private double  m_tx             = 0.0;   // Error horizontal (grados) — para rotación
    private double  m_ty             = 0.0;   // Error vertical (grados)   — para distancia
    private double  m_ta             = 0.0;   // Área del target (%) — opcional/filtrado futuro
    private double  m_distance       = 0.0;   // Distancia calculada al target (metros)
    private double  m_idealAngleDeg  = 0.0;   // Ángulo del hood según ShotMap (grados)

    // ─────────────────────────────────────────────────────────────────────────
    // CONSTRUCTOR
    // ─────────────────────────────────────────────────────────────────────────
    public limelightTurret() {
        // Sin hardware propio — solo lectura de NetworkTables vía LimelightHelpers
    }

    // ─────────────────────────────────────────────────────────────────────────
    // GETTERS — para consumo desde las torretas
    // ─────────────────────────────────────────────────────────────────────────

    /** ¿Hay un AprilTag válido en frame? */
    public boolean hasTarget() {
        return m_hasTarget;
    }

    /**
     * Error horizontal al target (grados).
     * Rango: -29.8° a +29.8°. Positivo = target a la derecha.
     * Usado por mobileTurretSubsystem para rotar hasta tx ≈ 0.
     */
    public double getTX() {
        return m_tx;
    }

    /**
     * Distancia al target calculada por trigonometría (metros).
     * Válida solo si hasTarget() == true.
     */
    public double getDistance() {
        return m_distance;
    }

    /**
     * Ángulo ideal del hood según la distancia al target (grados).
     * Resultado de kShotMap.get(distance). Válido solo si hasTarget() == true.
     */
    public double getIdealHoodAngleDeg() {
        return m_idealAngleDeg;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // CÁLCULO DE VOLTAJES — listos para usar en las torretas
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Voltaje para el motor de ROTACIÓN del turret móvil.
     *
     * Nula el error horizontal (tx → 0) con control proporcional.
     * Devuelve 0 si no hay target.
     *
     * @param maxVolts Voltaje máximo permitido (kRotationSpeed * 12.0)
     * @return Voltaje a aplicar al motor de rotación
     */
    public double getRotationVoltage(double maxVolts) {
        if (!m_hasTarget) return 0.0;
        double volts = m_tx * VisionConstants.kTrackingP * 12.0;
        return clamp(volts, -maxVolts, maxVolts);
    }

    /**
     * Voltaje para el motor de ÁNGULO DEL HOOD (ambas torretas).
     *
     * Convierte el ángulo ideal del ShotMap a revoluciones del motor
     * usando la relación de transmisión y aplica control proporcional al error.
     *
     * @param currentRevs   Revoluciones actuales del motor (de getPosition())
     * @param gearRatio     Relación de transmisión (kHoodGearRatio de la torreta)
     * @param maxVolts      Voltaje máximo permitido (kAngleSpeed * 12.0)
     * @param isHomed       El motor debe estar homeado para moverse
     * @return Voltaje a aplicar al motor del hood
     */
    public double getHoodVoltage(double currentRevs, double gearRatio,
                                 double maxVolts, boolean isHomed) {
        if (!m_hasTarget || !isHomed) return 0.0;

        double targetRevs = (m_idealAngleDeg / 360.0) * gearRatio;
        double error      = targetRevs - currentRevs;
        return clamp(error * maxVolts, -maxVolts, maxVolts);
    }

    /**
     * Ángulo ideal del hood convertido a revoluciones del motor.
     * Útil para mostrar en dashboard o comparar contra la posición actual.
     *
     * @param gearRatio Relación de transmisión del hood
     * @return Revoluciones objetivo del motor
     */
    public double getIdealHoodRevs(double gearRatio) {
        return (m_idealAngleDeg / 360.0) * gearRatio;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // CÁLCULO INTERNO — DISTANCIA
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Calcula la distancia horizontal al target usando geometría de cámara inclinada.
     *
     * Fórmula:
     *   angleToGoalRad = toRadians(kMountAngleDeg + ty)
     *   distance = (kTargetHeightMeters - kCameraHeightMeters) / tan(angleToGoalRad)
     *
     * kMountAngleDeg: ángulo de inclinación de la cámara desde la horizontal.
     * ty: offset vertical reportado por Limelight (positivo = arriba).
     * kTargetHeightMeters: altura del centro del AprilTag sobre el suelo.
     * kCameraHeightMeters: altura del lente de la cámara sobre el suelo.
     *
     * @param ty Valor getTY() del Limelight (grados)
     * @return Distancia en metros. Devuelve 0 si el ángulo calculado es <= 0.
     */
    private double calculateDistance(double ty) {
        double angleRad = Math.toRadians(VisionConstants.kMountAngleDeg + ty);
        if (angleRad <= 0.0) return 0.0; // Evita división por cero o valor negativo
        return (VisionConstants.kTargetHeightMeters - kCameraHeightMeters)
               / Math.tan(angleRad);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // UTILIDAD
    // ─────────────────────────────────────────────────────────────────────────
    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    // ─────────────────────────────────────────────────────────────────────────
    // PERIODIC — lectura única del Limelight por ciclo de 20 ms
    // ─────────────────────────────────────────────────────────────────────────
    @Override
    public void periodic() {

        // ── 1. Leer Limelight ─────────────────────────────────────────────
        m_hasTarget = LimelightHelpers.getTV(VisionConstants.limelightName);
        m_tx        = LimelightHelpers.getTX(VisionConstants.limelightName);
        m_ty        = LimelightHelpers.getTY(VisionConstants.limelightName);
        m_ta        = LimelightHelpers.getTA(VisionConstants.limelightName);

        // ── 2. Calcular distancia y ángulo ideal ──────────────────────────
        if (m_hasTarget) {
            m_distance      = calculateDistance(m_ty);
            m_idealAngleDeg = ShootingConfigs.kShotMap.get(m_distance);
        } else {
            m_distance      = 0.0;
            m_idealAngleDeg = 0.0;
        }

        // ── 3. SmartDashboard ─────────────────────────────────────────────
        SmartDashboard.putBoolean("Vision/Has Target",        m_hasTarget);
        SmartDashboard.putNumber ("Vision/TX (deg)",          m_tx);
        SmartDashboard.putNumber ("Vision/TY (deg)",          m_ty);
        SmartDashboard.putNumber ("Vision/TA (%)",            m_ta);
        SmartDashboard.putNumber ("Vision/Distance (m)",      m_distance);
        SmartDashboard.putNumber ("Vision/Ideal Angle (deg)", m_idealAngleDeg);
        SmartDashboard.putNumber ("Vision/Camera Height (m)", kCameraHeightMeters);
    }
}