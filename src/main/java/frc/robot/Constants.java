package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import com.pathplanner.lib.util.FlippingUtil;

public final class Constants {

    // ─────────────────────────────────────────────────────────────────────────
    // MODO DEL ROBOT
    // ─────────────────────────────────────────────────────────────────────────
    public enum ModoRobot { REAL, REPLAY, SIM }
    public static final ModoRobot currentMode = ModoRobot.REAL;

    // ─────────────────────────────────────────────────────────────────────────
    // CONTROLES (OI)
    // ─────────────────────────────────────────────────────────────────────────
    public static final class OIConstants {
        public static final int kDriverControllerPort    = 0;
        public static final int kMechanismsControllerPort = 1;
        public static final double kDeadband = 0.10;

        // Ejes del Driver
        public static final int kDriverYAxis   = 1;
        public static final int kDriverXAxis   = 0;
        public static final int kDriverRotAxis = 4;

        // Botones del Driver
        public static final int kDriverFieldOrientedButton = 3;
        public static final int kDriverResetHeading        = 2;
        public static final int kDriverResetElevator       = 1;
        public static final int kDriverHandOutButton       = 6;
        public static final int kDriverHandInButton        = 5;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // AUTONOMOUS
    // ─────────────────────────────────────────────────────────────────────────
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond                = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared  = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond        = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController     = 1;
        public static final double kPYController     = 1;
        public static final double kPThetaController = 1;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // MÓDULOS DE SWERVE
    // ─────────────────────────────────────────────────────────────────────────
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters     = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio     = 1 / 6.75;
        public static final double kTurningMotorGearRatio   = 1 / 21.4286;

        public static final double kDriveEncoderRot2Meter       = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad       = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

        public static final double kPTurning = 0.5;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // DRIVE / SWERVE
    // ─────────────────────────────────────────────────────────────────────────
    public static final class DriveConstants {

        // Geometría del chasis
        public static final double kTrackWidth = Units.inchesToMeters(27.4); // Distancia entre ruedas izq/der
        public static final double kWheelBase  = Units.inchesToMeters(27);   // Distancia entre ruedas front/back

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d( kWheelBase / 2, -kTrackWidth / 2),  // Front Right
            new Translation2d( kWheelBase / 2,  kTrackWidth / 2),  // Front Left
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),  // Back Right
            new Translation2d(-kWheelBase / 2,  kTrackWidth / 2)   // Back Left
        );

        // ── IDs CAN: Front Left (Drive / Steer / CANcoder) ──
        public static final int kFrontLeftDriveMotorPort            = 1;
        public static final int kFrontLeftTurningMotorPort          = 2;
        public static final int kFrontLeftDriveAbsoluteEncoderPort  = 3;

        // ── IDs CAN: Front Right (Drive / Steer / CANcoder) ──
        public static final int kFrontRightDriveMotorPort           = 4;
        public static final int kFrontRightTurningMotorPort         = 5;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 6;

        // ── IDs CAN: Back Left (Drive / Steer / CANcoder) ──
        public static final int kBackLeftDriveMotorPort             = 7;
        public static final int kBackLeftTurningMotorPort           = 8;
        public static final int kBackLeftDriveAbsoluteEncoderPort   = 9;

        // ── IDs CAN: Back Right (Drive / Steer / CANcoder) ──
        public static final int kBackRightDriveMotorPort            = 10;
        public static final int kBackRightTurningMotorPort          = 11;
        public static final int kBackRightDriveAbsoluteEncoderPort  = 12;

        // ── Inversiones de Turning Encoders ──
        public static final boolean kFrontLeftTurningEncoderReversed  = false;
        public static final boolean kBackLeftTurningEncoderReversed   = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed  = false;

        // ── Inversiones de Drive Encoders ──
        public static final boolean kFrontLeftDriveEncoderReversed  = false;
        public static final boolean kBackLeftDriveEncoderReversed   = false;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed  = true;

        // ── Inversiones de Absolute Encoders ──
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed  = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed   = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed  = false;

        // ── Offsets de Absolute Encoders (calibración de ruedas) ──
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad  = -0.500000;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad   =  0.497314;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad =  0.497070;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad  =  0.491699;

        // ── Velocidades máximas ──
        public static final double kPhysicalMaxSpeedMetersPerSecond         = 3;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond              = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond      = kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond        = 0.5;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2;

        // Factor de limitación de velocidad (e.g. modo preciso)
        public static final double kSpeedLimitFactor = 0.9;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // INTAKE
    // ─────────────────────────────────────────────────────────────────────────
    public static final class IntakeConstants {
        public static final int kIntakeMotorId   = 21;
        public static final int kExtensorMotorId = 20;

        public static final double kIntakeSpeed   = 0.9;
        public static final double kExtensorSpeed = 0.9;

        // ── State Machine ──────────────────────────────────────────────────
        public enum State {
            RETRACTED,  // Extension at 0, rollers off
            EXTENDED,   // Extension at max, rollers spinning inward
            INDEXING,   // Extension retracting, rollers hold piece
            OUTTAKE     // Rollers reversed for ejection
        }

        public static final double  kGearRatio         = 5.0 / 1.0;
        public static final boolean kMotorInverted      = true;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // INDEXER
    // ─────────────────────────────────────────────────────────────────────────
    public static final class IndexerConstants {
        public static final int kMotorRightId  = 24;
        public static final int kMotorLeftId   = 23;
        public static final int kRollerMotorId = 22;

        public static final double  kIndexerSpeed  = 0.9;
        public static final double  kRollerSpeed   = 0.9;

        // ── Y-Valve Routing Modes ──────────────────────────────────────────
        // Controls left/right mecanum shaft directions to route balls
        public enum RoutingMode {
            BOTH_SHOOTERS,  // Left=Inward,  Right=Inward  — split to both shooters
            TURRET_ONLY,    // Left=Inward,  Right=Outward — 100% to turret
            FIXED_ONLY,     // Left=Outward, Right=Inward  — 100% to fixed shooter
            HOLD,           // Left=Stop,    Right=Stop    — micro-stow during flywheel recovery
            REVERSE         // Left=Outward, Right=Outward — jam clearance / ball ejection
        }

        public static final boolean kInvertLeft  = false;
        public static final boolean kInvertRight = true; // Giran encontrados
    }

    // ─────────────────────────────────────────────────────────────────────────
    // FEEDER
    // ─────────────────────────────────────────────────────────────────────────
    public static final class FeederConstants {
        public static final int    kFeederMotorId      = 25;
        public static final double kFeederSpeed        = 0.9;
        public static final double kShootDelaySeconds  = 0.5; // Esperar que shooter llegue a RPM
    }

    // ─────────────────────────────────────────────────────────────────────────
    // FIXED TURRET
    // ─────────────────────────────────────────────────────────────────────────
    public static final class FixedTurretConstants {
        public static final int kAngleMotorId   = 29;
        public static final int kShooterMotorId = 30;

        public static final double kAngleSpeed   = 0.5;
        public static final double kShooterSpeed = 0.9;

        // ── Gear ratio: motor revs -> grados del hood ─────────────────────
        public static final double kGearRatio = 54.0 / 20.0; // 2.7 revs de motor por rev de salida

        // ── Rango de movimiento del hood (grados) ─────────────────────────
        public static final double kHoodMinDeg = 15.0; // Ángulo mínimo — tiro cercano / plano
        public static final double kHoodMaxDeg = 55.0; // Ángulo máximo — tiro lejano

        // ── Rango del hood en revoluciones del motor (para soft limits) ───
        // Fórmula: grados / 360 * kGearRatio
        public static final double kHoodMinRevs = (kHoodMinDeg / 360.0) * kGearRatio; // ~0.1125
        public static final double kHoodMaxRevs = (kHoodMaxDeg / 360.0) * kGearRatio; // ~0.4125
    }

    // ─────────────────────────────────────────────────────────────────────────
    // MOBILE TURRET
    // ─────────────────────────────────────────────────────────────────────────
    public static final class MobileTurretConstants {
        public static final int kRotationMotorId = 26;
        public static final int kAngleMotorId    = 27;
        public static final int kShooterMotorId  = 28;

        public static final double kRotationSpeed = 0.9;
        public static final double kAngleSpeed    = 0.5;
        public static final double kShooterSpeed  = 0.9;

        // ── Gear ratio del hood: motor revs -> grados ─────────────────────
        // TODO: confirmar mecánica en robot — placeholder igual que FixedTurret
        public static final double kHoodGearRatio = 54.0 / 20.0;

        // ── Rango de movimiento del hood (grados) ─────────────────────────
        public static final double kHoodMinDeg = 15.0; // Ángulo mínimo — tiro cercano / plano
        public static final double kHoodMaxDeg = 60.0; // Ángulo máximo — tiro lejano (5° más que fixed)

        // ── Rango del hood en revoluciones del motor (para soft limits) ───
        public static final double kHoodMinRevs = (kHoodMinDeg / 360.0) * kHoodGearRatio; // ~0.1125
        public static final double kHoodMaxRevs = (kHoodMaxDeg / 360.0) * kHoodGearRatio; // ~0.45

        // ── Soft Limits de rotación (360° continuo) ───────────────────────
        public static final double kRotationMinDeg = -180.0;
        public static final double kRotationMaxDeg =  180.0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // CLIMBER
    // ─────────────────────────────────────────────────────────────────────────
    public static final class ClimberConstants {
        public static final int    kClimberMotorId = 31;
        public static final double kMaxSpeed       = 0.9;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // VISION
    // ─────────────────────────────────────────────────────────────────────────
    public static final class VisionConstants {
        public static final String limelightName = "limelight";

        // Montaje de la cámara
        public static final double kMountAngleDeg      = 25.0;
        public static final double kLimelightAngle     = 3.0;   // Ángulo de montaje (legacy)
        public static final double kLimelightHeight    = 10.0;  // Altura de la cámara (pulgadas)
        public static final double kTargetHeight       = 12.0;  // Altura del target (pulgadas)
        public static final double kTargetHeightMeters = 2.6;   // Altura del AprilTag (metros)

        // Tracking de AprilTags
        public static final double kTrackingP      = 0.05;  // Ganancia PID para rotación
        public static final double kTargetDistance = 0.4;   // Distancia objetivo al tag (metros)
        public static final double kDistanceDeadband = 0.05; // Zona muerta de distancia (metros)
        public static final double kDistanceP      = 1.0;   // Ganancia PID para avance/retroceso
        public static final double kMaxForwardSpeed = 0.5;  // Velocidad máxima de avance (m/s)

        // Offsets de alineación
        public static final double kTargetXOffset   = 20.0;
        public static final double xyStdDevCoefficient = 0.2;

        public enum Direction { Left, Right }

        // PID genérico de visión (legacy)
        public static final double kPController = 0.2;
        public static final double kIController = 0.001;
        public static final double kDController = 0.001;

        // ── Feature Flags ──────────────────────────────────────────────────
        // Toggle odometry vs vision tracking for turret aiming.
        // Competition default: false (odometry). Enable if odometry drifts.
        public static final boolean kUseTurretVisionTracking = false;

        // Std deviation weights — lower = more trust in vision measurement
        // Higher confidence when multiple AprilTags are visible
        public static final double kSingleTagStdDev  = 0.9; // 1 tag visible
        public static final double kMultiTagStdDev   = 0.3; // 2+ tags visible
    }

    // ─────────────────────────────────────────────────────────────────────────
    // SHOT MAP (Distancia -> Ángulo de disparo)
    // ─────────────────────────────────────────────────────────────────────────
    public static final class ShootingConfigs {
        public static final InterpolatingDoubleTreeMap kShotMap = new InterpolatingDoubleTreeMap();

        static {
            // Distancia (m) -> Ángulo del hood (grados)
            kShotMap.put(1.0, 25.0);
            kShotMap.put(3.0, 45.0);
            kShotMap.put(5.0, 58.0);
            // Agregar más puntos conforme prueben en pista
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // DRIVE MODE CONSTANTS
    // Speed caps and feature flags per driving mode
    // ─────────────────────────────────────────────────────────────────────────
    public static final class DriveModeConstants {

        // ── MODE B — BOMBER ───────────────────────────────────────────────
        // Maximum scoring output. Both shooters. Slowed for precision.
        public static final double  kBomberDriveSpeed       = 0.60; // 60% — precision orbit
        public static final double  kBomberRotSpeed         = 0.70; // 70% — fine aiming
        public static final double  kBomberShooterSpeedCap  = 0.70; // 70% — prevents re-spin slip
        public static final boolean kBomberOrbitEnabled     = true;
        public static final boolean kBomberClimberEnabled   = true;
        public static final boolean kBomberFixedEnabled     = true;
        public static final boolean kBomberTurretEnabled    = true;
        public static final boolean kBomberAutoHoodEnabled  = true;

        // ── MODE S — STRIKER ──────────────────────────────────────────────
        // Mid-field control and feeding. High mobility. Turret only.
        public static final double  kStrikerDriveSpeed      = 0.75; // 75% — faster movement
        public static final double  kStrikerRotSpeed        = 1.00; // 100% — full rotation
        public static final double  kStrikerShooterSpeedCap = 0.50; // 50% — gentle feed speed
        public static final boolean kStrikerOrbitEnabled    = false;
        public static final boolean kStrikerClimberEnabled  = false; // Save battery
        public static final boolean kStrikerFixedEnabled    = false; // Off entirely
        public static final boolean kStrikerTurretEnabled   = true;
        public static final boolean kStrikerAutoHoodEnabled = true;

        // ── MODE I — INTERCEPTOR ──────────────────────────────────────────
        // Full-speed offensive disruption. Ball stealing. No orbit.
        public static final double  kInterceptorDriveSpeed      = 1.00; // 100% — max speed
        public static final double  kInterceptorRotSpeed        = 1.00; // 100%
        public static final double  kInterceptorShooterSpeedCap = 0.75; // 75% — long-range shots
        public static final boolean kInterceptorOrbitEnabled    = false; // Movement priority
        public static final boolean kInterceptorClimberEnabled  = false; // Never climb
        public static final boolean kInterceptorFixedEnabled    = false; // Conditional (stationary only)
        public static final boolean kInterceptorTurretEnabled   = true;  // Opportunistic
        public static final boolean kInterceptorAutoHoodEnabled = true;

        // ── Helpers — lookup por modo activo ─────────────────────────────
        public static double getDriveSpeed(frc.robot.commands.mechanisms.SuperstructureCommand.DriveMode mode) {
            return switch (mode) {
                case BOMBER      -> kBomberDriveSpeed;
                case STRIKER     -> kStrikerDriveSpeed;
                case INTERCEPTOR -> kInterceptorDriveSpeed;
            };
        }

        public static double getRotSpeed(frc.robot.commands.mechanisms.SuperstructureCommand.DriveMode mode) {
            return switch (mode) {
                case BOMBER      -> kBomberRotSpeed;
                case STRIKER     -> kStrikerRotSpeed;
                case INTERCEPTOR -> kInterceptorRotSpeed;
            };
        }

        public static double getShooterCap(frc.robot.commands.mechanisms.SuperstructureCommand.DriveMode mode) {
            return switch (mode) {
                case BOMBER      -> kBomberShooterSpeedCap;
                case STRIKER     -> kStrikerShooterSpeedCap;
                case INTERCEPTOR -> kInterceptorShooterSpeedCap;
            };
        }

        public static boolean isOrbitEnabled(frc.robot.commands.mechanisms.SuperstructureCommand.DriveMode mode) {
            return switch (mode) {
                case BOMBER      -> kBomberOrbitEnabled;
                case STRIKER     -> kStrikerOrbitEnabled;
                case INTERCEPTOR -> kInterceptorOrbitEnabled;
            };
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // CURRENT LIMITS
    // ─────────────────────────────────────────────────────────────────────────
    public static final class CurrentLimits {
        // POLICY: Supply current only — no stator limits used anywhere.
        //
        // NO LIMIT (SupplyCurrentLimitEnable = false):
        //   • Both flywheels   (Kraken X60, IDs 28 & 30)
        //   • Chassis Drive    (4x Kraken X60, IDs 1/4/7/10)
        //   • Chassis Steer    (4x Kraken X44, IDs 2/5/8/11)
        //   • Climber          (Kraken X44, ID 31)
        //
        // 20A SUPPLY (SupplyCurrentLimit = kMechanismSupply):
        //   • Intake roller & extensor (IDs 20/21)
        //   • Indexer x3 (IDs 22/23/24)
        //   • Feeder (ID 25)
        //   • Turret rotation & hood (IDs 26/27)
        //   • Fixed hood (ID 29)
        public static final int     kMechanismSupply      = 20;    // Amps — todo lo demás
        public static final boolean kMechanismLimitEnable = true;  // Enable for mechanisms
        public static final boolean kNoLimitEnable        = false;  // Disable for shooters/drive/climber
    }

  public static final class FieldConstants {

        public static final double fieldBorderMargin = 0.01;
        public static final double fieldLength       = 17.29; // Metros
        public static final double fieldWidth        = 7.78;
        public static final double startingLineX     = Units.inchesToMeters(299.438);

        /* ── Poses de alineación BLUE (set 1) ── */
        public static final Pose2d[] alignBluePose = {
            new Pose2d(2.950, 4.1,   new Rotation2d(Units.degreesToRadians( 180))), // A
            new Pose2d(2.950, 3.860, new Rotation2d(Units.degreesToRadians( 180))), // B
            new Pose2d(3.549, 2.709, new Rotation2d(Units.degreesToRadians(-120))), // C
            new Pose2d(3.871, 2.553, new Rotation2d(Units.degreesToRadians(-120))), // D
            new Pose2d(5.109, 2.650, new Rotation2d(Units.degreesToRadians( -60))), // E
            new Pose2d(5.382, 2.787, new Rotation2d(Units.degreesToRadians( -60))), // F
            new Pose2d(6.320, 3.860, new Rotation2d(Units.degreesToRadians(   0))), // G
            new Pose2d(6.320, 4.150, new Rotation2d(Units.degreesToRadians(   0))), // H
            new Pose2d(5.490, 5.450, new Rotation2d(Units.degreesToRadians(  60))), // I
            new Pose2d(5.128, 5.478, new Rotation2d(Units.degreesToRadians(  60))), // J
            new Pose2d(3.920, 5.430, new Rotation2d(Units.degreesToRadians( 120))), // K
            new Pose2d(3.520, 5.270, new Rotation2d(Units.degreesToRadians( 120))), // L
        };

        /* ── Poses de alineación RED (set 1, espejadas) ── */
        public static final Pose2d[] alignRedPose = {
            FlippingUtil.flipFieldPose(alignBluePose[0]),  // A
            FlippingUtil.flipFieldPose(alignBluePose[1]),  // B
            FlippingUtil.flipFieldPose(alignBluePose[2]),  // C
            FlippingUtil.flipFieldPose(alignBluePose[3]),  // D
            FlippingUtil.flipFieldPose(alignBluePose[4]),  // E
            FlippingUtil.flipFieldPose(alignBluePose[5]),  // F
            FlippingUtil.flipFieldPose(alignBluePose[6]),  // G
            FlippingUtil.flipFieldPose(alignBluePose[7]),  // H
            FlippingUtil.flipFieldPose(alignBluePose[8]),  // I
            FlippingUtil.flipFieldPose(alignBluePose[9]),  // J
            FlippingUtil.flipFieldPose(alignBluePose[10]), // K
            FlippingUtil.flipFieldPose(alignBluePose[11]), // L
        };

        /* ── Poses de alineación BLUE (set 2, más cercanas) ── */
        public static final Pose2d[] blueSidePositions = {
            new Pose2d(3.300, 4.150, new Rotation2d(Units.degreesToRadians( 180))), // A
            new Pose2d(3.300, 3.860, new Rotation2d(Units.degreesToRadians( 180))), // B
            new Pose2d(3.700, 3.010, new Rotation2d(Units.degreesToRadians(-120))), // C
            new Pose2d(3.990, 2.870, new Rotation2d(Units.degreesToRadians(-120))), // D
            new Pose2d(4.963, 2.835, new Rotation2d(Units.degreesToRadians( -60))), // E
            new Pose2d(5.250, 3.030, new Rotation2d(Units.degreesToRadians( -60))), // F
            new Pose2d(5.750, 3.860, new Rotation2d(Units.degreesToRadians(   0))), // G
            new Pose2d(5.750, 4.150, new Rotation2d(Units.degreesToRadians(   0))), // H
            new Pose2d(5.255, 5.039, new Rotation2d(Units.degreesToRadians(  60))), // I
            new Pose2d(4.963, 5.205, new Rotation2d(Units.degreesToRadians(  60))), // J
            new Pose2d(4.017, 5.205, new Rotation2d(Units.degreesToRadians( 120))), // K
            new Pose2d(3.750, 5.100, new Rotation2d(Units.degreesToRadians( 120))), // L
        };

        /* ── Poses de alineación RED (set 2, espejadas) ── */
        public static final Pose2d[] redSidePositions = {
            FlippingUtil.flipFieldPose(blueSidePositions[0]),  // A
            FlippingUtil.flipFieldPose(blueSidePositions[1]),  // B
            FlippingUtil.flipFieldPose(blueSidePositions[2]),  // C
            FlippingUtil.flipFieldPose(blueSidePositions[3]),  // D
            FlippingUtil.flipFieldPose(blueSidePositions[4]),  // E
            FlippingUtil.flipFieldPose(blueSidePositions[5]),  // F
            FlippingUtil.flipFieldPose(blueSidePositions[6]),  // G
            FlippingUtil.flipFieldPose(blueSidePositions[7]),  // H
            FlippingUtil.flipFieldPose(blueSidePositions[8]),  // I
            FlippingUtil.flipFieldPose(blueSidePositions[9]),  // J
            FlippingUtil.flipFieldPose(blueSidePositions[10]), // K
            FlippingUtil.flipFieldPose(blueSidePositions[11]), // L
        };
    }
}