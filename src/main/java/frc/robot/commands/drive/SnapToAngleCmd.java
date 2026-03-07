package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

/**
 * SnapToAngleCmd — rota el chassis a un ángulo cardinal mientras permite
 * movimiento libre en X/Y.
 *
 * Trigger: D-Pad (0°=Up, 90°=Right, 180°=Down, 270°=Left)
 *
 * Ejemplo en RobotContainer:
 *   new POVButton(ctrl, 0)  .whileTrue(new SnapToAngleCmd(m_drive, xSup, ySup,   0));
 *   new POVButton(ctrl, 90) .whileTrue(new SnapToAngleCmd(m_drive, xSup, ySup,  90));
 *   new POVButton(ctrl, 180).whileTrue(new SnapToAngleCmd(m_drive, xSup, ySup, 180));
 *   new POVButton(ctrl, 270).whileTrue(new SnapToAngleCmd(m_drive, xSup, ySup, 270));
 */
public class SnapToAngleCmd extends Command {

    private static final double kSnapP       = 5.0;
    private static final double kSnapI       = 0.0;
    private static final double kSnapD       = 0.1;
    private static final double kMaxAngVel   = Math.PI * 2; // rad/s
    private static final double kMaxAngAccel = Math.PI * 4; // rad/s²
    private static final double kTolerance   = 0.03;        // rad (~1.7°)

    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_xSupplier;
    private final DoubleSupplier m_ySupplier;
    private final double         m_targetDeg;

    private final ProfiledPIDController m_snapPID = new ProfiledPIDController(
        kSnapP, kSnapI, kSnapD,
        new TrapezoidProfile.Constraints(kMaxAngVel, kMaxAngAccel)
    );

    private final SlewRateLimiter m_xLimiter =
        new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    private final SlewRateLimiter m_yLimiter =
        new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

    public SnapToAngleCmd(DriveSubsystem drive,
                          DoubleSupplier xSupplier,
                          DoubleSupplier ySupplier,
                          double targetDegrees) {
        m_drive      = drive;
        m_xSupplier  = xSupplier;
        m_ySupplier  = ySupplier;
        m_targetDeg  = targetDegrees;

        m_snapPID.enableContinuousInput(-Math.PI, Math.PI);
        m_snapPID.setTolerance(kTolerance);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_snapPID.reset(m_drive.getHeading().getRadians());
    }

    @Override
    public void execute() {
        double currentRad = m_drive.getHeading().getRadians();
        double targetRad  = Math.toRadians(m_targetDeg);
        double rot = m_snapPID.calculate(currentRad, targetRad);
        rot = Math.max(-kMaxAngVel, Math.min(kMaxAngVel, rot));

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

    /** Termina cuando el robot está dentro de tolerancia Y joysticks en reposo. */
    @Override
    public boolean isFinished() { return false; } // whileTrue lo cancela al soltar el D-Pad

    private double applyDeadband(SlewRateLimiter limiter, double raw, double maxVal) {
        double db = Math.abs(raw) > OIConstants.kDeadband ? raw : 0.0;
        return limiter.calculate(db) * maxVal;
    }
}
