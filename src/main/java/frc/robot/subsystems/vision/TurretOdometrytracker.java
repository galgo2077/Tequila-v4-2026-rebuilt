package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

import java.util.function.Supplier;

/**
 * TurretOdometryTracker — calcula el ángulo que debe tener el turret
 * basado en la pose del robot y la posición conocida del objetivo en el campo.
 *
 * ALGORITMO (Sección 9 del spec):
 *   1. Obtener robot pose de SwerveDrivePoseEstimator
 *   2. Obtener posición del target de FieldConstants
 *   3. Calcular vector Translation2d de robot → target
 *   4. atan2(dy, dx) → ángulo requerido en coordenadas de campo
 *   5. Restar heading del robot → ángulo relativo al turret
 *   6. Unwrap para minimizar rotación (camino más corto)
 *   7. Convertir a revoluciones de motor → listo para MotionMagicVoltage
 *
 * USO:
 *   Llamar getTargetTurretRevs() cada ciclo en mobileTurretSubsystem
 *   cuando kUseTurretVisionTracking == false (modo odometría).
 *
 *   Cuando kUseTurretVisionTracking == true, usar limelightTurret.getTX()
 *   como corrección adicional encima de este resultado.
 */
public class TurretOdometryTracker {

    // ── Inyección de dependencias ─────────────────────────────────────────────
    private final Supplier<Pose2d>    m_poseSupplier;
    private final Supplier<Rotation2d> m_headingSupplier;

    // ── Estado ────────────────────────────────────────────────────────────────
    private double m_lastTargetRevs = 0.0;

    // ── Límite de rotación del turret (±180° = ±0.5 rotaciones del motor * gear ratio) ──
    // Cambiar si el turret tiene topes mecánicos
    private static final double kMaxTurretRevs  =  10.0; // TODO: confirmar en robot
    private static final double kMinTurretRevs  = -10.0;

    public TurretOdometryTracker(Supplier<Pose2d> poseSupplier,
                                  Supplier<Rotation2d> headingSupplier) {
        m_poseSupplier    = poseSupplier;
        m_headingSupplier = headingSupplier;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // API PÚBLICA
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Devuelve la posición de motor (en revoluciones) que el turret debe alcanzar
     * para apuntar al objetivo activo según la alianza.
     *
     * @param gearRatio  Relación de reducción del turret (motor revs / output revs)
     * @return           Posición en revoluciones de motor, lista para MotionMagicVoltage
     */
    public double getTargetTurretRevs(double gearRatio) {
        Pose2d robot    = m_poseSupplier.get();
        Translation2d target = getActiveTarget(robot);

        // Paso 3 — vector robot → target
        double dx = target.getX() - robot.getX();
        double dy = target.getY() - robot.getY();

        // Paso 4 — ángulo de campo requerido
        double fieldAngleRad = Math.atan2(dy, dx);

        // Paso 5 — ángulo relativo al robot (turret-relative)
        double robotHeadingRad  = m_headingSupplier.get().getRadians();
        double turretRelativeRad = fieldAngleRad - robotHeadingRad;

        // Paso 6 — unwrap al rango [-π, π]
        turretRelativeRad = unwrapAngle(turretRelativeRad);

        // Convertir ángulo (rad) → revoluciones de output → revoluciones de motor
        double outputRevs = turretRelativeRad / (2.0 * Math.PI);
        double motorRevs  = outputRevs * gearRatio;

        // Clamp a límites del turret
        motorRevs = Math.max(kMinTurretRevs, Math.min(kMaxTurretRevs, motorRevs));

        m_lastTargetRevs = motorRevs;
        publishDashboard(robot, target, fieldAngleRad, turretRelativeRad, motorRevs);

        return motorRevs;
    }

    /**
     * Distancia euclidiana al objetivo activo (metros).
     * Útil para el ShotMap aunque no haya visión.
     */
    public double getDistanceToTarget() {
        Pose2d robot     = m_poseSupplier.get();
        Translation2d t  = getActiveTarget(robot);
        return robot.getTranslation().getDistance(t);
    }

    /** Último valor calculado, sin recalcular. */
    public double getLastTargetRevs() {
        return m_lastTargetRevs;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // TARGET SELECTION — azul vs rojo, goal vs alliance zone
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Devuelve la posición del objetivo según la alianza activa.
     * El target es el centro del Speaker (posición A+B promediada).
     *
     * En modo STRIKER, el target debería ser la zona de alianza —
     * pasar el boolean al constructor si se necesita separar.
     */
    private Translation2d getActiveTarget(Pose2d robot) {
        boolean isBlue = DriverStation.getAlliance()
            .map(a -> a == DriverStation.Alliance.Blue)
            .orElse(true);

        Pose2d[] poses = isBlue
            ? FieldConstants.alignBluePose
            : FieldConstants.alignRedPose;

        // Centro del Speaker = promedio de pose A y pose B
        return new Translation2d(
            (poses[0].getX() + poses[1].getX()) / 2.0,
            (poses[0].getY() + poses[1].getY()) / 2.0
        );
    }

    // ─────────────────────────────────────────────────────────────────────────
    // HELPERS
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Normaliza un ángulo al rango [-π, π].
     * Garantiza que el turret siempre tome el camino más corto.
     */
    private static double unwrapAngle(double rad) {
        while (rad >  Math.PI) rad -= 2.0 * Math.PI;
        while (rad < -Math.PI) rad += 2.0 * Math.PI;
        return rad;
    }

    private void publishDashboard(Pose2d robot, Translation2d target,
                                   double fieldAngleRad, double turretRelRad,
                                   double motorRevs) {
        SmartDashboard.putNumber("OdoTracker/Robot X (m)",         robot.getX());
        SmartDashboard.putNumber("OdoTracker/Robot Y (m)",         robot.getY());
        SmartDashboard.putNumber("OdoTracker/Target X (m)",        target.getX());
        SmartDashboard.putNumber("OdoTracker/Target Y (m)",        target.getY());
        SmartDashboard.putNumber("OdoTracker/Field Angle (deg)",   Units.radiansToDegrees(fieldAngleRad));
        SmartDashboard.putNumber("OdoTracker/Turret Rel (deg)",    Units.radiansToDegrees(turretRelRad));
        SmartDashboard.putNumber("OdoTracker/Target Motor Revs",   motorRevs);
        SmartDashboard.putNumber("OdoTracker/Distance (m)",        getDistanceToTarget());
    }
}