package frc.robot.commands.mechanisms;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.mechanisms.SuperstructureCommand;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * ShootDualCmd — prepara ambos shooters y dispara cuando el guard se cumple.
 * Trigger: Hold RB (Bomber)
 *
 * Flujo:
 *   1. Entra a PREPPING_DUAL (flywheels ramping, hoods apuntando)
 *   2. Espera hasta que isMobileShooterReady() && isFixedShooterReady()
 *   3. Entra a SHOOTING_DUAL (feeder activo)
 *   4. SuperstructureCommand auto-retorna a IDLE después del kShootDelaySeconds
 *
 * Si el operador suelta RB antes de que el guard pase,
 * el finallyDo() envía IDLE y cancela el prep.
 */
public class ShootDualCmd extends Command {

    private final SuperstructureCommand m_super;

    public ShootDualCmd(SuperstructureCommand superstructure) {
        m_super = superstructure;
        // Sin addRequirements — SuperstructureCommand ya tiene los subsistemas
    }

    @Override
    public void initialize() {
        m_super.requestState(SuperstructureCommand.State.PREPPING_DUAL);
    }

    @Override
    public void execute() {
        // Transicionar a SHOOTING cuando ambos shooters estén listos
        if (m_super.getState() == SuperstructureCommand.State.PREPPING_DUAL
                && m_super.isMobileShooterReady()
                && m_super.isFixedShooterReady()) {
            m_super.requestState(SuperstructureCommand.State.SHOOTING_DUAL);
        }
    }

    @Override
    public boolean isFinished() {
        // Termina cuando SuperstructureCommand completó el disparo y volvió a IDLE
        return m_super.getState() == SuperstructureCommand.State.IDLE;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_super.requestState(SuperstructureCommand.State.IDLE);
        }
    }
}
