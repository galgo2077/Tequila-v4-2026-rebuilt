package frc.robot.commands.mechanisms;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.mechanisms.SuperstructureCommand;

/**
 * PrepShootersCmd — enciende flywheels y mueve hoods al ángulo ideal
 * SIN activar el feeder. Pre-calentamiento antes de llegar al punto de disparo.
 * Trigger: Hold X
 * Al soltar → IDLE (usar con whileTrue en RobotContainer).
 */
public class PrepShootersCmd extends InstantCommand {
    public PrepShootersCmd(SuperstructureCommand superstructure) {
        super(() -> superstructure.requestState(SuperstructureCommand.State.PREPPING_DUAL));
    }
}
