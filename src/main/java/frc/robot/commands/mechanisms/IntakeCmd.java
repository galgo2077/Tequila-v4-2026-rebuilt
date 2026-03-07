package frc.robot.commands.mechanisms;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.mechanisms.SuperstructureCommand;

/**
 * IntakeCmd — despliega el intake y corre los rollers.
 * Trigger: Hold RT
 * Al soltar → IDLE (usar con whileTrue en RobotContainer).
 */
public class IntakeCmd extends InstantCommand {
    public IntakeCmd(SuperstructureCommand superstructure) {
        super(() -> superstructure.requestState(SuperstructureCommand.State.INTAKING));
    }
}
