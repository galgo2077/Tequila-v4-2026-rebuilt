package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    public DriveSubsystem() {}
    public Pose2d        getPose()                           { return new Pose2d(); }
    public void          resetOdometry(Pose2d pose)          {}
    public void          resetPose(Pose2d pose)              {}
    public Rotation2d    getRotation2d()                     { return new Rotation2d(); }
    public double        getHeading()                        { return 0.0; }
    public void          zeroHeading()                       {}
    public ChassisSpeeds getRobotRelativeSpeeds()            { return new ChassisSpeeds(); }
    public void          driveRobotRelative(ChassisSpeeds s) {}
    public void          drive(double x, double y, double rot, boolean fieldRelative) {}
    public void          stopModules()                       {}
    @Override public void periodic() {}
}