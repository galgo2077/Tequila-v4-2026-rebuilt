package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.mechanisms.SuperstructureCommand.DriveMode;
import frc.robot.commands.mechanisms.SuperstructureCommand;

/**
 * SetDriveModeCmd — cambia entre Bomber / Striker / Interceptor.
 *
 * Trigger: Menu / Select buttons
 *
 * Ejemplo en RobotContainer:
 *   new JoystickButton(ctrl, 7).onTrue(new SetDriveModeCmd(m_super, DriveMode.BOMBER));
 *   new JoystickButton(ctrl, 8).onTrue(new SetDriveModeCmd(m_super, DriveMode.STRIKER));
 *   new JoystickButton(ctrl, 9).onTrue(new SetDriveModeCmd(m_super, DriveMode.INTERCEPTOR));
 */
public class SetDriveModeCmd extends InstantCommand {

    public SetDriveModeCmd(SuperstructureCommand superstructure, DriveMode mode) {
        super(() -> superstructure.setDriveMode(mode));
        // Sin addRequirements — no toca ningún subsistema de hardware directamente
    }
}
