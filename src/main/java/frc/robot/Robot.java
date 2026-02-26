// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Main robot class. Methods correspond to robot modes.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  // Robot startup initialization
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  // Called periodically (20ms) regardless of mode
  @Override
  public void robotPeriodic() {
    // Run the scheduler
    CommandScheduler.getInstance().run();
  }

  // Called once when entering Disabled mode
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  // Runs the autonomous command
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Schedule the autonomous command
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  // Called periodically during autonomous
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // Stop autonomous command on teleop start
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  // Called periodically during operator control
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels running commands at start of test mode
    CommandScheduler.getInstance().cancelAll();
  }

  // Called periodically during test mode
  @Override
  public void testPeriodic() {
  }

  // Called once when simulation starts
  @Override
  public void simulationInit() {
  }

  // Called periodically during simulation
  @Override
  public void simulationPeriodic() {
  }
}
