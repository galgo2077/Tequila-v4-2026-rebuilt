package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
// TODO: import frc.robot.subsystems.DriveSubsystem; (add when DriveSubsystem is created)

import java.util.function.DoubleSupplier;

/**
 * OrbitTargetCmd — el chassis siempre apunta al objetivo mientras el driver
 * se mueve libremente en X/Y.
 *
 * El turret se ocupa de apuntar finamente con Limelight — este comando solo
 * mantiene el chassis orientado para que el turret tenga rango libre de rotación.
 *
 * Trigger: Hold LB (modo Bomber)
 *
 * NOTA: El target a apuntar es el centro del Speaker del alliance color activo.
 * Cambiar getTargetPose() si el target cambia según la alianza.
 */
public class OrbitTargetCmd extends Command {

    private static final double kOrbitP       = 4.0;
    private static final double kOrbitI       = 0.0;
    private static final double kOrbitD       = 0.1;
    private static final double kOrbitTolerance = 0.05; // rad

    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_xSupplier;
    private final DoubleSupplier m_ySupplier;
    private final boolean        m_blueAlliance;

    private final PIDController m_headingPID =
        new PIDController(kOrbitP, kOrbitI, kOrbitD);

    private final SlewRateLimiter m_xLimiter =
        new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    private final SlewRateLimiter m_yLimiter =
        new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

    public OrbitTargetCmd(DriveSubsystem drive,
                          DoubleSupplier xSupplier,
                          DoubleSupplier ySupplier,
                          boolean blueAlliance) {
        m_drive        = drive;
        m_xSupplier    = xSupplier;
        m_ySupplier    = ySupplier;
        m_blueAlliance = blueAlliance;

        m_headingPID.enableContinuousInput(-Math.PI, Math.PI);
        m_headingPID.setTolerance(kOrbitTolerance);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_headingPID.reset();
    }

    @Override
    public void execute() {
        // Posición actual del robot
        Translation2d robotPos = m_drive.getPose().getTranslation();

        // Posición del target en campo
        Translation2d target = getTargetTranslation();

        // Ángulo que debe tener el chassis para apuntar al target
        double targetAngleRad = Math.atan2(
            target.getY() - robotPos.getY(),
            target.getX() - robotPos.getX()
        );

        double currentAngleRad = m_drive.getHeading().getRadians();
        double rot = m_headingPID.calculate(currentAngleRad, targetAngleRad);
        rot = Math.max(-DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond,
              Math.min( DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond, rot));

        double xSpeed = applyDeadband(m_xLimiter, m_xSupplier.getAsDouble(),
            DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
        double ySpeed = applyDeadband(m_yLimiter, m_ySupplier.getAsDouble(),
            DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);

        m_drive.drive(xSpeed, ySpeed, rot, true);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() { return false; }

    // ── Helpers ──────────────────────────────────────────────────────────────

    private Translation2d getTargetTranslation() {
        // Posición del Speaker según alianza — usar el centro (promedio de A y B)
        if (m_blueAlliance) {
            return new Translation2d(
                (FieldConstants.alignBluePose[0].getX() + FieldConstants.alignBluePose[1].getX()) / 2.0,
                (FieldConstants.alignBluePose[0].getY() + FieldConstants.alignBluePose[1].getY()) / 2.0
            );
        } else {
            return new Translation2d(
                (FieldConstants.alignRedPose[0].getX() + FieldConstants.alignRedPose[1].getX()) / 2.0,
                (FieldConstants.alignRedPose[0].getY() + FieldConstants.alignRedPose[1].getY()) / 2.0
            );
        }
    }

    private double applyDeadband(SlewRateLimiter limiter, double raw, double maxVal) {
        double db = Math.abs(raw) > OIConstants.kDeadband ? raw : 0.0;
        return limiter.calculate(db) * maxVal;
    }
}
