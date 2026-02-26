// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Robot-wide constants. Use inner classes for specific subsystems.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerChasis = 0;
    public static final int kDriverControllerIntake = 1;
  }

  public static class IntakeConstants {

    // ids
    public static final int kIntakeMotor = 1;
    public static final int kExtensorMotor = 2;

    // speeds
    public static final double kIntakeSpeedPositive = 0.9;
    public static final double kIntakeSpeedNegative = -0.9;

    public static final double kExtensorSpeedPositive = 0.9;
    public static final double kExtensorSpeedNegative = -0.9;
  }

  public static class IndexerConstants {

    // ids
    public static final int kIndexerMotorRight = 4;
    public static final int kIndexerMotorLeft = 5;
    public static final int kRollerMotor = 6;

    // speeds
    public static final double kIndexerSpeedPositive = 0.9;// both indexers
    public static final double kIndexerSpeedNegative = -0.9;

    public static final double kRollerSpeedPositive = 0.9;
    public static final double kRollerSpeedNegative = -0.9;

  }

  public static class FeederConstants {

    // ids
    public static final int kFeederMotor = 7;

    // speeds
    public static final double kFeederSpeedPositive = 0.9;
    public static final double kFeederSpeedNegative = -0.9;
  }

}
