package frc.robot.subsystems.mechanism;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IndexerConstants.RoutingMode;
import frc.robot.subsystems.mechanism.io.IndexerIO;
import frc.robot.subsystems.mechanism.io.IndexerIO.IndexerIOInputs;
import java.util.function.BooleanSupplier;

/**
 * Indexer subsystem — Y-valve dual mecanum shaft routing.
 * Directs game pieces to either or both shooters.
 *
 * Routing logic lives here; raw voltage calls go through IndexerIO.
 */
public class IndexerSubsystem extends SubsystemBase {

    // ── IO layer ──────────────────────────────────────────────────────────────
    private final IndexerIO       io;
    private final IndexerIOInputs inputs = new IndexerIOInputs();

    // ── Constructor ───────────────────────────────────────────────────────────
    public IndexerSubsystem(IndexerIO io) {
        this.io = io;
    }

    // ── Voltage setters ───────────────────────────────────────────────────────

    public void setRollerVoltage(double volts)                          { io.setRollerVoltage(volts); }
    public void setIndexerVoltage(double rightVolts, double leftVolts)  { io.setShaftVoltages(leftVolts, rightVolts); }
    public void stop()                                                  { io.stopAll(); }

    // ── Y-valve routing ───────────────────────────────────────────────────────

    /**
     * Apply a Y-valve routing mode.
     * The roller runs at full speed in all non-HOLD/REVERSE modes.
     */
    public void setRoutingMode(RoutingMode mode) {
        double speed = IndexerConstants.kIndexerSpeed * 12.0;
        switch (mode) {
            case BOTH_SHOOTERS -> {
                io.setRollerVoltage(IndexerConstants.kRollerSpeed * 12.0);
                io.setShaftVoltages(speed, speed);   // both inward
            }
            case TURRET_ONLY -> {
                io.setRollerVoltage(IndexerConstants.kRollerSpeed * 12.0);
                io.setShaftVoltages(speed, -speed);  // left inward, right outward
            }
            case FIXED_ONLY -> {
                io.setRollerVoltage(IndexerConstants.kRollerSpeed * 12.0);
                io.setShaftVoltages(-speed, speed);  // left outward, right inward
            }
            case HOLD -> {
                io.setRollerVoltage(0);
                io.setShaftVoltages(0, 0);
            }
            case REVERSE -> {
                io.setRollerVoltage(-IndexerConstants.kRollerSpeed * 12.0);
                io.setShaftVoltages(-speed, -speed);
            }
        }
    }

    // ── Sensor accessors ──────────────────────────────────────────────────────

    public double getRollerVelocityRPS()  { return inputs.rollerVelocityRPS; }
    public double getRollerCurrentAmps()  { return inputs.rollerCurrentAmps; }

    // ── Command factories ─────────────────────────────────────────────────────

    /** Full manual control — roller and both shafts independently. */
    public Command runIndexerCommand(
            BooleanSupplier rollerIn,
            BooleanSupplier rollerOut,
            BooleanSupplier indexIn,
            BooleanSupplier indexOut) {
        return this.run(() -> {
            if      (rollerIn.getAsBoolean())  setRollerVoltage( IndexerConstants.kRollerSpeed * 12.0);
            else if (rollerOut.getAsBoolean()) setRollerVoltage(-IndexerConstants.kRollerSpeed * 12.0);
            else                               setRollerVoltage(0);

            double shaftV = IndexerConstants.kIndexerSpeed * 12.0;
            if      (indexIn.getAsBoolean())  setIndexerVoltage( shaftV,  shaftV);
            else if (indexOut.getAsBoolean()) setIndexerVoltage(-shaftV, -shaftV);
            else                              setIndexerVoltage(0, 0);
        }).finallyDo(interrupted -> stop());
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        SmartDashboard.putNumber("Indexer/Roller RPS",    inputs.rollerVelocityRPS);
        SmartDashboard.putNumber("Indexer/Roller Amps",   inputs.rollerCurrentAmps);
        SmartDashboard.putNumber("Indexer/Left Amps",     inputs.leftCurrentAmps);
        SmartDashboard.putNumber("Indexer/Right Amps",    inputs.rightCurrentAmps);
        SmartDashboard.putBoolean("Indexer/Roller Conn",  inputs.rollerConnected);
        SmartDashboard.putBoolean("Indexer/Left Conn",    inputs.leftConnected);
        SmartDashboard.putBoolean("Indexer/Right Conn",   inputs.rightConnected);
    }
}
