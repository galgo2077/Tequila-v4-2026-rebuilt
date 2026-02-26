package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeSubsystem; // Asegúrate de que la clase empiece con Mayúscula en su archivo

public class IntakeCommand extends Command {

    private final intakeSubsystem m_subsystem;

    private final BooleanSupplier m_intakein;
    private final BooleanSupplier m_intakeout;

    private final BooleanSupplier m_extensorin;
    private final BooleanSupplier m_extensorout;

    public IntakeCommand(intakeSubsystem subsystem, BooleanSupplier intakein, BooleanSupplier intakeout, BooleanSupplier extensorin, BooleanSupplier extensorout) {
        m_subsystem = subsystem;
        
        m_intakein = intakein;
        m_intakeout = intakeout;
        m_extensorin = extensorin;
        m_extensorout = extensorout;

        // Es vital registrar el subsistema para que dos comandos no lo usen a la vez
        addRequirements(m_subsystem);
    }

    @Override
    public void execute() {
        // Le pasamos el valor booleano del botón a la lógica del subsistema
        m_subsystem.intake(m_intakein.getAsBoolean(), m_intakeout.getAsBoolean());
        m_subsystem.Extensor(m_extensorin.getAsBoolean(), m_extensorout.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {
        // Cuando el comando se detiene o se interrumpe, apagamos el motor
        m_subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        // Normalmente devuelve false para que el comando corra mientras el botón esté presionado
        return false;
    }
}