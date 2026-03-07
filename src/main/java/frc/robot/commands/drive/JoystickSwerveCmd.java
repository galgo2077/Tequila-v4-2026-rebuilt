package frc.robot.commands.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * JoystickSwerveCmd — comando de conducción por defecto con swerve.
 *
 * Usa deadband, slew rate limiter y permite togglear field-relative.
 */
public class JoystickSwerveCmd extends Command {

    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_xSupplier;
    private final DoubleSupplier m_ySupplier;
    private final DoubleSupplier m_rotSupplier;
    private final BooleanSupplier m_fieldRelativeSupplier;

    private final SlewRateLimiter m_xLimiter =
        new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    private final SlewRateLimiter m_yLimiter =
        new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    private final SlewRateLimiter m_rotLimiter =
        new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    public JoystickSwerveCmd(DriveSubsystem drive,
                             DoubleSupplier xSupplier,
                             DoubleSupplier ySupplier,
                             DoubleSupplier rotSupplier,
                             BooleanSupplier fieldRelativeSupplier) {
        m_drive = drive;
        m_xSupplier = xSupplier;
        m_ySupplier = ySupplier;
        m_rotSupplier = rotSupplier;
        m_fieldRelativeSupplier = fieldRelativeSupplier;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double xSpeed = applyDeadbandAndLimit(m_xLimiter,
            m_xSupplier.getAsDouble(),
            DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);

        double ySpeed = applyDeadbandAndLimit(m_yLimiter,
            m_ySupplier.getAsDouble(),
            DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);

        double rot = applyDeadbandAndLimit(m_rotLimiter,
            m_rotSupplier.getAsDouble(),
            DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);

        m_drive.drive(xSpeed, ySpeed, rot, m_fieldRelativeSupplier.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() { return false; }

    private double applyDeadbandAndLimit(SlewRateLimiter limiter, double raw, double maxVal) {
        double deadbanded = Math.abs(raw) > OIConstants.kDeadband ? raw : 0.0;
        return limiter.calculate(deadbanded) * maxVal;
    }
}
