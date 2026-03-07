package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * ZeroHeadingCmd — resetea el yaw del giroscopio al heading actual del campo.
 *
 * Trigger: Start button
 *
 * Usar cuando el robot se coloca manualmente apuntando hacia el frente del campo
 * y el giroscopio se ha derivado durante el match.
 */
public class ZeroHeadingCmd extends InstantCommand {

    public ZeroHeadingCmd(DriveSubsystem drive) {
        super(drive::zeroHeading, drive);
    }
}
