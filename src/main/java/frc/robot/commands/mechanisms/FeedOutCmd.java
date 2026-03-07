package frc.robot.commands.mechanism;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * FeedOutCmd — enruta piezas hacia la zona de alianza.
 * Trigger: Hold B (Striker)
 * Al soltar → IDLE (usar con whileTrue en RobotContainer).
 */
public class FeedOutCmd extends InstantCommand {
    public FeedOutCmd(SuperstructureCommand superstructure) {
        super(() -> superstructure.requestState(SuperstructureCommand.State.FEEDING_OUT));
    }
}
