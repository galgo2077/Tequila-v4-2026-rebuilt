package frc.robot.commands.mechanism;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * ShootTurretCmd — prepara solo el turret móvil y dispara cuando está listo.
 * Trigger: Hold RB (Striker / Interceptor)
 */
public class ShootTurretCmd extends Command {

    private final SuperstructureCommand m_super;

    public ShootTurretCmd(SuperstructureCommand superstructure) {
        m_super = superstructure;
    }

    @Override
    public void initialize() {
        m_super.requestState(SuperstructureCommand.State.PREPPING_TURRET);
    }

    @Override
    public void execute() {
        if (m_super.getState() == SuperstructureCommand.State.PREPPING_TURRET
                && m_super.isMobileShooterReady()) {
            m_super.requestState(SuperstructureCommand.State.SHOOTING_TURRET);
        }
    }

    @Override
    public boolean isFinished() {
        return m_super.getState() == SuperstructureCommand.State.IDLE;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) m_super.requestState(SuperstructureCommand.State.IDLE);
    }
}
