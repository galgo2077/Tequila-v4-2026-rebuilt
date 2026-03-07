package frc.robot.commands.mechanisms;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.mechanisms.SuperstructureCommand;

/**
 * ClimbCmd — activa el modo de escalada (solo en BOMBER, climber homeado).
 * Trigger: Hold Y (Bomber)
 *
 * El joystick del climber sigue siendo leído por SuperstructureCommand
 * en su executeClimbing() a través del DoubleSupplier inyectado en el constructor.
 */
public class ClimbCmd extends InstantCommand {
    public ClimbCmd(SuperstructureCommand superstructure) {
        super(() -> superstructure.requestState(SuperstructureCommand.State.CLIMBING));
    }
}
