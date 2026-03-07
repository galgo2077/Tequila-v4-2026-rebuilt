package frc.robot.subsystems.mechanism;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MobileTurretConstants;
import frc.robot.Constants.ShootingConfigs;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.mechanism.io.MobileTurretIO;
import frc.robot.subsystems.mechanism.io.MobileTurretIO.MobileTurretIOInputs;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Mobile (360°) Turret subsystem — rotation, hood, and flywheel.
 * Both rotation and hood require current-spike homing before use.
 * Vision tracking (Limelight) is polled in periodic() and cached for commands.
 */
public class mobileTurretSubsystem extends SubsystemBase {

    // ── IO layer ──────────────────────────────────────────────────────────────
    private final MobileTurretIO       io;
    private final MobileTurretIOInputs inputs = new MobileTurretIOInputs();

    // ── Homing parameters ─────────────────────────────────────────────────────
    private static final double kAngleHomingVolts              = -1.5;
    private static final double kAngleHomingCurrentThreshold   = 8.0;
    private static final double kRotationHomingVolts           = -1.5;
    private static final double kRotationHomingCurrentThreshold = 10.0;

    // ── Homing state ──────────────────────────────────────────────────────────
    private boolean m_angleHomed     = false;
    private boolean m_rotationHomed  = false;
    private boolean m_angleHoming    = false;
    private boolean m_rotationHoming = false;

    // ── Vision cache (updated each periodic) ─────────────────────────────────
    private boolean m_hasTarget      = false;
    private double  m_cachedTx       = 0.0;
    private double  m_cachedDistance = 0.0;

    // ── Constructor ───────────────────────────────────────────────────────────
    public mobileTurretSubsystem(MobileTurretIO io) {
        this.io = io;
    }

    // ── Public action methods ─────────────────────────────────────────────────

    public void setAngleVoltage(double volts)    { io.setAngleVoltage(volts); }
    public void setShooterVoltage(double volts)  { io.setShooterVoltage(volts); }
    public void setRotationVoltage(double volts) { io.setRotationVoltage(volts); }
    public void stop()                           { io.stopAll(); }

    // ── Sensor accessors ──────────────────────────────────────────────────────

    public double  getAngleRevs()           { return inputs.anglePositionRevs; }
    public double  getShooterVelocityRPS()  { return inputs.shooterVelocityRPS; }
    public boolean isAngleHomed()           { return m_angleHomed; }
    public boolean isRotationHomed()        { return m_rotationHomed; }
    public boolean isFullyHomed()           { return m_angleHomed && m_rotationHomed; }

    // ── Vision accessors ──────────────────────────────────────────────────────

    public boolean hasTarget()            { return m_hasTarget; }
    public double  getTargetTX()          { return m_cachedTx; }
    public double  getDistanceToTarget()  { return m_cachedDistance; }

    // ── Auto-aim voltage calculations ─────────────────────────────────────────

    public double getAutoRotationVoltage() {
        if (!m_hasTarget) return 0.0;
        double maxV = MobileTurretConstants.kRotationSpeed * 12.0;
        return Math.max(-maxV, Math.min(maxV, m_cachedTx * VisionConstants.kTrackingP * 12.0));
    }

    public double getAutoAngleVoltage() {
        if (!m_hasTarget || !m_angleHomed) return 0.0;
        double targetDeg  = ShootingConfigs.kShotMap.get(m_cachedDistance);
        double targetRevs = (targetDeg / 360.0) * MobileTurretConstants.kHoodGearRatio;
        double error      = targetRevs - inputs.anglePositionRevs;
        double maxV       = MobileTurretConstants.kAngleSpeed * 12.0;
        return Math.max(-maxV, Math.min(maxV, error * maxV));
    }

    // ── Homing helpers ────────────────────────────────────────────────────────

    private void completeAngleHoming() {
        io.resetAngleEncoder();
        io.enableAngleSoftLimits(true, true);
        m_angleHomed  = true;
        m_angleHoming = false;
        io.stopAll();
        System.out.println("[MobileTurret] Angle homing complete.");
    }

    private void completeRotationHoming() {
        io.resetRotationEncoder();
        io.enableRotationSoftLimits(true, true);
        m_rotationHomed  = true;
        m_rotationHoming = false;
        io.stopAll();
        System.out.println("[MobileTurret] Rotation homing complete.");
    }

    // ── Command factories ─────────────────────────────────────────────────────

    public Command homeAngleCommand() {
        return this.runOnce(() -> { m_angleHomed = false; m_angleHoming = true; })
            .andThen(
                this.run(() -> setAngleVoltage(kAngleHomingVolts))
                    .until(() -> inputs.angleCurrentAmps > kAngleHomingCurrentThreshold)
            )
            .andThen(this.runOnce(this::completeAngleHoming));
    }

    public Command homeRotationCommand() {
        return this.runOnce(() -> { m_rotationHomed = false; m_rotationHoming = true; })
            .andThen(
                this.run(() -> setRotationVoltage(kRotationHomingVolts))
                    .until(() -> inputs.rotationCurrentAmps > kRotationHomingCurrentThreshold)
            )
            .andThen(this.runOnce(this::completeRotationHoming));
    }

    /** Sequential homing: angle first, then rotation. */
    public Command homeAllCommand() {
        return homeAngleCommand().andThen(homeRotationCommand());
    }

    /** Manual control. Blocked until homed. */
    public Command runManualCommand(DoubleSupplier angleVolts, DoubleSupplier rotationVolts, BooleanSupplier shoot) {
        return this.run(() -> {
            setAngleVoltage(m_angleHomed    ? angleVolts.getAsDouble()    * 12.0 : 0.0);
            setRotationVoltage(m_rotationHomed ? rotationVolts.getAsDouble() * 12.0 : 0.0);
            setShooterVoltage(shoot.getAsBoolean() ? MobileTurretConstants.kShooterSpeed * 12.0 : 0.0);
        }).finallyDo(interrupted -> stop());
    }

    /** Auto-aim using Limelight TX + shot map. */
    public Command runAutoAimCommand(BooleanSupplier shoot) {
        return this.run(() -> {
            if (!isFullyHomed()) return;
            setRotationVoltage(m_hasTarget ? getAutoRotationVoltage() : 0.0);
            setAngleVoltage(m_hasTarget    ? getAutoAngleVoltage()    : 0.0);
            setShooterVoltage(MobileTurretConstants.kShooterSpeed * 12.0);
        }).finallyDo(interrupted -> stop());
    }

    /** Spin up shooter and pre-aim without firing. */
    public Command runSpinUpCommand() {
        return this.run(() -> {
            setShooterVoltage(MobileTurretConstants.kShooterSpeed * 12.0);
            if (isFullyHomed() && m_hasTarget) {
                setRotationVoltage(getAutoRotationVoltage());
                setAngleVoltage(getAutoAngleVoltage());
            }
        }).finallyDo(interrupted -> stop());
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        // Refresh Limelight vision data
        m_hasTarget = LimelightHelpers.getTV(VisionConstants.limelightName);
        m_cachedTx  = LimelightHelpers.getTX(VisionConstants.limelightName);
        if (m_hasTarget) {
            double ty             = LimelightHelpers.getTY(VisionConstants.limelightName);
            double angleToGoalRad = Math.toRadians(VisionConstants.kMountAngleDeg + ty);
            m_cachedDistance      = (VisionConstants.kTargetHeightMeters
                                     - (VisionConstants.kMountAngleDeg * 0.0174533))
                                    / Math.tan(angleToGoalRad);
        } else {
            m_cachedDistance = 0.0;
        }

        SmartDashboard.putNumber("MobileTurret/Angle Revs",     inputs.anglePositionRevs);
        SmartDashboard.putNumber("MobileTurret/Angle Amps",     inputs.angleCurrentAmps);
        SmartDashboard.putNumber("MobileTurret/Shooter RPS",    inputs.shooterVelocityRPS);
        SmartDashboard.putNumber("MobileTurret/Shooter Amps",   inputs.shooterCurrentAmps);
        SmartDashboard.putNumber("MobileTurret/Rotation Revs",  inputs.rotationPositionRevs);
        SmartDashboard.putNumber("MobileTurret/Rotation Amps",  inputs.rotationCurrentAmps);
        SmartDashboard.putBoolean("MobileTurret/Angle Homed",   m_angleHomed);
        SmartDashboard.putBoolean("MobileTurret/Rot Homed",     m_rotationHomed);
        SmartDashboard.putBoolean("MobileTurret/Has Target",    m_hasTarget);
        SmartDashboard.putNumber("MobileTurret/TX",             m_cachedTx);
        SmartDashboard.putNumber("MobileTurret/Distance (m)",   m_cachedDistance);
        SmartDashboard.putNumber("MobileTurret/Ideal Angle",
            m_hasTarget ? ShootingConfigs.kShotMap.get(m_cachedDistance) : 0.0);
    }
}
