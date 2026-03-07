package frc.robot.subsystems.mechanism;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.mechanism.io.IntakeIO;
import frc.robot.subsystems.mechanism.io.IntakeIO.IntakeIOInputs;
import java.util.function.BooleanSupplier;

/**
 * Intake subsystem — roller (intake/outtake) + rack-and-pinion extensor.
 * Extensor uses current-spike homing to find its zero at the retracted hard stop.
 */
public class intakeSubsystem extends SubsystemBase {

    // ── IO layer ──────────────────────────────────────────────────────────────
    private final IntakeIO       io;
    private final IntakeIOInputs inputs = new IntakeIOInputs();

    // ── Extensor travel (motor revolutions) ──────────────────────────────────
    private static final double kExtensorDeployed         = 15.0;  // Measure on robot
    private static final double kHomingVoltage            = -1.5;
    private static final double kHomingCurrentThreshold   = 8.0;   // Amps

    // ── Homing state ──────────────────────────────────────────────────────────
    private boolean m_isHomed  = false;
    private boolean m_isHoming = false;

    // ── Constructor ───────────────────────────────────────────────────────────
    public intakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    // ── Public action methods ─────────────────────────────────────────────────

    public void setIntakeVoltage(double volts)   { io.setRollerVoltage(volts); }
    public void setExtensorVoltage(double volts) { io.setExtensorVoltage(volts); }
    public void stop()                           { io.stopAll(); }

    // ── Sensor accessors ──────────────────────────────────────────────────────

    public double  getExtensorPositionRevs() { return inputs.extensorPositionRevs; }
    public double  getExtensorCurrentAmps()  { return inputs.extensorCurrentAmps; }
    public boolean isHomed()                 { return m_isHomed; }

    // ── Homing helper (called from inside homingCommand) ──────────────────────
    private void completeHoming() {
        io.resetExtensorEncoder();
        io.enableExtensorSoftLimits(true, true);
        m_isHomed  = true;
        m_isHoming = false;
        io.stopAll();
        System.out.println("[Intake] Homing complete. Encoder zeroed.");
    }

    // ── Command factories ─────────────────────────────────────────────────────

    /**
     * Drive extensor against retracted hard stop until current spikes,
     * then zero the encoder and enable soft limits.
     * Run once at match start.
     */
    public Command homingCommand() {
        return this.runOnce(() -> {
            m_isHomed  = false;
            m_isHoming = true;
            System.out.println("[Intake] Starting homing...");
        })
        .andThen(
            this.run(() -> setExtensorVoltage(kHomingVoltage))
                .until(() -> inputs.extensorCurrentAmps > kHomingCurrentThreshold)
        )
        .andThen(this.runOnce(this::completeHoming));
    }

    /** Manual control: roller + extensor in/out. Extensor blocked until homed. */
    public Command runIntakeExtensorCommand(
            BooleanSupplier intakeIn,
            BooleanSupplier extensorIn,
            BooleanSupplier extensorOut) {
        return this.run(() -> {
            setIntakeVoltage(intakeIn.getAsBoolean() ? IntakeConstants.kIntakeSpeed * 12.0 : 0.0);

            if (!m_isHomed) { setExtensorVoltage(0); return; }

            if (extensorIn.getAsBoolean())        setExtensorVoltage( IntakeConstants.kExtensorSpeed * 12.0);
            else if (extensorOut.getAsBoolean())   setExtensorVoltage(-IntakeConstants.kExtensorSpeed * 12.0);
            else                                   setExtensorVoltage(0);
        }).finallyDo(interrupted -> stop());
    }

    /** One-touch deploy or retract using a simple P controller. */
    public Command goToPositionCommand(boolean deploy) {
        double targetRevs = deploy ? kExtensorDeployed : 0.0;
        return this.run(() -> {
            if (!m_isHomed) return;
            double error   = targetRevs - getExtensorPositionRevs();
            double voltage = Math.max(-6.0, Math.min(6.0, error * 0.4));
            setExtensorVoltage(voltage);
        })
        .until(() -> Math.abs(targetRevs - getExtensorPositionRevs()) < 0.3)
        .finallyDo(interrupted -> setExtensorVoltage(0));
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        SmartDashboard.putNumber("Intake/Roller RPS",       inputs.rollerVelocityRPS);
        SmartDashboard.putNumber("Intake/Roller Amps",      inputs.rollerCurrentAmps);
        SmartDashboard.putNumber("Intake/Extensor Revs",    inputs.extensorPositionRevs);
        SmartDashboard.putNumber("Intake/Extensor Amps",    inputs.extensorCurrentAmps);
        SmartDashboard.putBoolean("Intake/Is Homed",        m_isHomed);
        SmartDashboard.putBoolean("Intake/Is Homing",       m_isHoming);
        SmartDashboard.putBoolean("Intake/Roller Connected", inputs.rollerConnected);
        SmartDashboard.putBoolean("Intake/Ext Connected",   inputs.extensorConnected);
    }
}
