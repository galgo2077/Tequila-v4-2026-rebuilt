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
  }

  public static class fixedTurretConstants {

    // ids
    public static final int kAngleMotorFixed = 8;
    public static final int kShooterFixedMotor = 9;

    // speeds
    public static final double kAngleSpeedPositiveFixed = 0.9;
    public static final double kAngleSpeedNegativeFixed = -0.9;

    public static final double kShooterSpeedPositiveFixed = 0.9;
  }

  public static class mobileTurretConstants {

    // ids
    public static final int kAngleMotorMobile = 10;
    public static final int kShooterMotorMobile = 11;
    public static final int kTurretMotorMobile = 12;

    // speeds
    public static final double kAngleSpeedPositiveMobile = 0.9;
    public static final double kAngleSpeedNegativeMobile = -0.9;

    public static final double kShooterSpeedPositiveMobile = 0.9;

    public static final double kTurretSpeedPositiveMobile = 0.9;
    public static final double kTurretSpeedNegativeMobile = -0.9;
  }

  public static class climberConstants {

    // ids
    public static final int kClimberMotor = 13;

    // speeds
    public static final double kClimberSpeedPositive = 0.9;
    public static final double kClimberSpeedNegative = -0.9;
  }

}
