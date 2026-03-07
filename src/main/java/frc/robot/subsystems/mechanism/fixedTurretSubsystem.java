package frc.robot.subsystems.mechanism;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FixedTurretConstants;
import frc.robot.Constants.ShootingConfigs;
import frc.robot.Constants.VisionConstants;

import frc.robot.subsystems.mechanism.io.FixedTurretIO;
import frc.robot.subsystems.mechanism.io.FixedTurretIO.FixedTurretIOInputs;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Fixed Shooter subsystem — hood (angle) + flywheel.
 * Always faces the same direction as the chassis.
 * Hood requires current-spike homing before use.
 */
public class fixedTurretSubsystem extends SubsystemBase {

    // ── IO layer ──────────────────────────────────────────────────────────────
    private final FixedTurretIO       io;
    private final FixedTurretIOInputs inputs = new FixedTurretIOInputs();

    // ── Homing parameters ─────────────────────────────────────────────────────
    private static final double kAngleHomingVolts            = -1.5;
    private static final double kAngleHomingCurrentThreshold = 8.0;

    // ── Homing state ──────────────────────────────────────────────────────────
    private boolean m_isHomed  = false;
    private boolean m_isHoming = false;

    // ── Vision cache ──────────────────────────────────────────────────────────
    private boolean m_hasTarget      = false;
    private double  m_cachedDistance = 0.0;

    // ── Constructor ───────────────────────────────────────────────────────────
    public fixedTurretSubsystem(FixedTurretIO io) {
        this.io = io;
    }

    // ── Public action methods ─────────────────────────────────────────────────

    public void setAngleVoltage(double volts)   { io.setAngleVoltage(volts); }
    public void setShooterVoltage(double volts) { io.setShooterVoltage(volts); }
    public void stop()                          { io.stopAll(); }

    // ── Sensor accessors ──────────────────────────────────────────────────────

    public double  getAngleRevs()          { return inputs.anglePositionRevs; }
    public double  getShooterVelocityRPS() { return inputs.shooterVelocityRPS; }
    public boolean isHomed()               { return m_isHomed; }
    public boolean hasTarget()             { return m_hasTarget; }
    public double  getDistanceToTarget()   { return m_cachedDistance; }

    // ── Auto-aim voltage calculation ──────────────────────────────────────────

    public double getAutoAngleVoltage() {
        if (!m_hasTarget || !m_isHomed) return 0.0;
        double targetDeg  = ShootingConfigs.kShotMap.get(m_cachedDistance);
        double targetRevs = (targetDeg / 360.0) * FixedTurretConstants.kGearRatio;
        double error      = targetRevs - inputs.anglePositionRevs;
        double maxV       = FixedTurretConstants.kAngleSpeed * 12.0;
        return Math.max(-maxV, Math.min(maxV, error * maxV));
    }

    // ── Homing helper ─────────────────────────────────────────────────────────

    private void completeHoming() {
        io.resetAngleEncoder();
        io.enableAngleSoftLimits(true, true);
        m_isHomed  = true;
        m_isHoming = false;
        io.stopAll();
        System.out.println("[FixedTurret] Homing complete. Encoder zeroed.");
    }

    // ── Command factories ─────────────────────────────────────────────────────

    /** Drive hood against hard stop, zero encoder, enable soft limits. */
    public Command homingCommand() {
        return this.runOnce(() -> { m_isHomed = false; m_isHoming = true; })
            .andThen(
                this.run(() -> setAngleVoltage(kAngleHomingVolts))
                    .until(() -> inputs.angleCurrentAmps > kAngleHomingCurrentThreshold)
            )
            .andThen(this.runOnce(this::completeHoming));
    }

    /** Manual control with joystick. Hood blocked until homed. */
    public Command runManualCommand(DoubleSupplier angleVolts, BooleanSupplier shoot) {
        return this.run(() -> {
            setAngleVoltage(m_isHomed ? angleVolts.getAsDouble() * 12.0 : 0.0);
            setShooterVoltage(shoot.getAsBoolean() ? FixedTurretConstants.kShooterSpeed * 12.0 : 0.0);
        }).finallyDo(interrupted -> stop());
    }

    /** Auto-aim hood using distance from shot map, shooter at full speed. */
    public Command runAutoAimCommand(BooleanSupplier shoot) {
        return this.run(() -> {
            if (m_isHomed) setAngleVoltage(getAutoAngleVoltage());
            setShooterVoltage(FixedTurretConstants.kShooterSpeed * 12.0);
        }).finallyDo(interrupted -> stop());
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        m_hasTarget = LimelightHelpers.getTV(VisionConstants.limelightName);
        if (m_hasTarget) {
            double ty             = LimelightHelpers.getTY(VisionConstants.limelightName);
            double angleToGoalRad = Math.toRadians(VisionConstants.kMountAngleDeg + ty);
            m_cachedDistance      = (VisionConstants.kTargetHeightMeters
                                     - (VisionConstants.kMountAngleDeg * 0.0174533))
                                    / Math.tan(angleToGoalRad);
        } else {
            m_cachedDistance = 0.0;
        }

        SmartDashboard.putNumber("FixedTurret/Angle Revs",     inputs.anglePositionRevs);
        SmartDashboard.putNumber("FixedTurret/Angle Amps",     inputs.angleCurrentAmps);
        SmartDashboard.putNumber("FixedTurret/Shooter RPS",    inputs.shooterVelocityRPS);
        SmartDashboard.putNumber("FixedTurret/Shooter Amps",   inputs.shooterCurrentAmps);
        SmartDashboard.putBoolean("FixedTurret/Is Homed",      m_isHomed);
        SmartDashboard.putBoolean("FixedTurret/Has Target",    m_hasTarget);
        SmartDashboard.putNumber("FixedTurret/Distance (m)",   m_cachedDistance);
        SmartDashboard.putNumber("FixedTurret/Ideal Angle",
            m_hasTarget ? ShootingConfigs.kShotMap.get(m_cachedDistance) : 0.0);
        SmartDashboard.putBoolean("FixedTurret/Angle Conn",    inputs.angleConnected);
        SmartDashboard.putBoolean("FixedTurret/Shooter Conn",  inputs.shooterConnected);
    }
}
