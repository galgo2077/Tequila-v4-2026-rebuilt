package frc.robot.commands.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

/**
 * JoystickDriveCmd — comando de conducción por defecto.
 *
 * Field-relative, con deadband, slew rate limiter y factor de velocidad
 * configurable según el modo de conducción.
 *
 * Programación en RobotContainer:
 *   m_drive.setDefaultCommand(new JoystickDriveCmd(
 *       m_drive,
 *       () -> -m_driverCtrl.getRawAxis(OIConstants.kDriverYAxis),
 *       () -> -m_driverCtrl.getRawAxis(OIConstants.kDriverXAxis),
 *       () -> -m_driverCtrl.getRawAxis(OIConstants.kDriverRotAxis),
 *       () -> m_super.getSpeedFactor()));
 */
public class JoystickDriveCmd extends Command {

    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_xSupplier;   // Adelante / atrás
    private final DoubleSupplier m_ySupplier;   // Izquierda / derecha
    private final DoubleSupplier m_rotSupplier; // Rotación
    private final DoubleSupplier m_speedFactor; // 0.0–1.0 según DriveMode

    // Slew rate limiters — suavizan los cambios bruscos de joystick
    private final SlewRateLimiter m_xLimiter   =
        new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    private final SlewRateLimiter m_yLimiter   =
        new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    private final SlewRateLimiter m_rotLimiter =
        new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    public JoystickDriveCmd(DriveSubsystem drive,
                             DoubleSupplier xSupplier,
                             DoubleSupplier ySupplier,
                             DoubleSupplier rotSupplier,
                             DoubleSupplier speedFactor) {
        m_drive       = drive;
        m_xSupplier   = xSupplier;
        m_ySupplier   = ySupplier;
        m_rotSupplier = rotSupplier;
        m_speedFactor = speedFactor;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double factor = m_speedFactor.getAsDouble();

        double xSpeed = applyDeadbandAndLimit(m_xLimiter,
            m_xSupplier.getAsDouble(),
            DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * factor);

        double ySpeed = applyDeadbandAndLimit(m_yLimiter,
            m_ySupplier.getAsDouble(),
            DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * factor);

        double rot = applyDeadbandAndLimit(m_rotLimiter,
            m_rotSupplier.getAsDouble(),
            DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * factor);

        m_drive.drive(xSpeed, ySpeed, rot, true /* field-relative */);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() { return false; }

    // ── Helper ───────────────────────────────────────────────────────────────
    private double applyDeadbandAndLimit(SlewRateLimiter limiter, double raw, double maxVal) {
        double deadbanded = Math.abs(raw) > OIConstants.kDeadband ? raw : 0.0;
        return limiter.calculate(deadbanded) * maxVal;
    }
}
