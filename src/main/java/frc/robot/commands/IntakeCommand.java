package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeSubsystem; // Ensure class names are capitalized

public class IntakeCommand extends Command {

    private final intakeSubsystem m_subsystem;

    private final BooleanSupplier m_intakein;

    private final BooleanSupplier m_extensorin;
    private final BooleanSupplier m_extensorout;

    public IntakeCommand(intakeSubsystem subsystem, BooleanSupplier intakein, BooleanSupplier extensorin,
            BooleanSupplier extensorout) {
        m_subsystem = subsystem;

        m_intakein = intakein;
        m_extensorin = extensorin;
        m_extensorout = extensorout;

        // Register subsystem requirement
        addRequirements(m_subsystem);
    }

    @Override
    public void execute() {
        // Pass button values to subsystem logic
        m_subsystem.intake(m_intakein.getAsBoolean());
        m_subsystem.Extensor(m_extensorin.getAsBoolean(), m_extensorout.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {
        // Stop motor when interrupted or finished
        m_subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        // Run while button is held
        //
        return false;
    }
}