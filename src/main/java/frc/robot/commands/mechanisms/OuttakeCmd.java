package frc.robot.commands.mechanisms;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.mechanisms.SuperstructureCommand;

/**
 * OuttakeCmd — reversa intake e indexer para expulsar piezas.
 * Trigger: Hold LT
 * Al soltar → IDLE (usar con whileTrue en RobotContainer).
 */
public class OuttakeCmd extends InstantCommand {
    public OuttakeCmd(SuperstructureCommand superstructure) {
        super(() -> superstructure.requestState(SuperstructureCommand.State.OUTTAKE));
    }
}
