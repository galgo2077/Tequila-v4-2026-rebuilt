package frc.robot.commands.drive;
import frc.robot.subsystems.drive.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;

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
