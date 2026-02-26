// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Robot-wide constants. Use inner classes for specific subsystems.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class IntakeConstants {

    public static final int kIntakeMotor = 1;
    public static final int kExtensorMotor = 2;
  }

  public static class IndexerConstants {

    public static final int kIndexerMotorRight = 4;
    public static final int kIndexerMotorLeft = 5;
    public static final int kRollerMotor = 6;
  }

  public static class FeederConstants {

    public static final int kFeederMotor = 7;
  }

}
