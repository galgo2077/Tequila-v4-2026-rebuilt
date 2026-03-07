package frc.robot.subsystems.mechanism;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.mechanism.io.FeederIO;
import frc.robot.subsystems.mechanism.io.FeederIO.FeederIOInputs;
import java.util.function.BooleanSupplier;

/**
 * Feeder subsystem — kicker wheel that pre-accelerates game pieces before
 * they reach the shooters.
 *
 * Hardware is fully abstracted through FeederIO. Inject FeederIOReal on the
 * robot or FeederIOSim during simulation/replay.
 */
public class feederSubsystem extends SubsystemBase {

    // ── IO layer ──────────────────────────────────────────────────────────────
    private final FeederIO         io;
    private final FeederIOInputs   inputs = new FeederIOInputs();

    // ── Constructor ───────────────────────────────────────────────────────────
    public feederSubsystem(FeederIO io) {
        this.io = io;
    }

    // ── Public action methods ─────────────────────────────────────────────────

    public void setFeederVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void stop() {
        io.stop();
    }

    // ── Sensor accessors (read from logged inputs) ────────────────────────────

    public double getVelocityRPS()    { return inputs.velocityRPS; }
    public double getCurrentAmps()    { return inputs.supplyCurrentAmps; }
    public boolean isConnected()      { return inputs.motorConnected; }

    // ── Command factories ─────────────────────────────────────────────────────

    /** Run feeder while button is held. */
    public Command runFeederCommand(BooleanSupplier feederIn) {
        return this.run(() ->
            setFeederVoltage(feederIn.getAsBoolean() ? FeederConstants.kFeederSpeed * 12.0 : 0.0)
        ).finallyDo(interrupted -> stop());
    }

    /** Reverse feeder (eject). */
    public Command reverseFeederCommand(BooleanSupplier feederOut) {
        return this.run(() ->
            setFeederVoltage(feederOut.getAsBoolean() ? -FeederConstants.kFeederSpeed * 12.0 : 0.0)
        ).finallyDo(interrupted -> stop());
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        SmartDashboard.putNumber("Feeder/Velocity RPS",   inputs.velocityRPS);
        SmartDashboard.putNumber("Feeder/Current Amps",   inputs.supplyCurrentAmps);
        SmartDashboard.putNumber("Feeder/Applied Volts",  inputs.appliedVolts);
        SmartDashboard.putBoolean("Feeder/Connected",     inputs.motorConnected);
    }
}
