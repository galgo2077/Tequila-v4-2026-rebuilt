package frc.robot.commands.mechanism;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * ClimbReleaseCmd — al soltar Y, bloquea el climber en la posición actual.
 *
 * SuperstructureCommand.executeClimbing() ya aplica voltaje 0 cuando el joystick
 * está en reposo. El motor del climber está en NeutralMode.Brake, así que
 * simplemente no enviar voltaje lo mantiene bloqueado.
 *
 * Este comando no cambia el estado — permanece en CLIMBING para que los soft
 * limits del climber sigan activos.
 *
 * Trigger: Release Y (onFalse del botón Y)
 */
public class ClimbReleaseCmd extends InstantCommand {
    public ClimbReleaseCmd(SuperstructureCommand superstructure) {
        // No cambia el estado — el joystick simplemente deja de dar input
        // y el brake mode mantiene la posición. Solo loggea para debugging.
        super(() -> System.out.println("[Climber] Bloqueado en posición actual."));
    }
}
