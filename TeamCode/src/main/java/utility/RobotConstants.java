package utility;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class RobotConstants {
    public static class Drivetrain {
        public static String frontLeftMotor = "frontLeftMotor"; //E0
        public static String backLeftMotor = "backLeftMotor"; // E1
        public static String frontRightMotor= "frontRightMotor"; // E2
        public static String backRightMotor = "backRightMotor"; // E3
    }

    public static class Robot {
        // Robot dimensions (inches)
        public static final double ROBOT_SIZE = 17; // Robot width/length (square)
        public static final double HALF_SIZE = ROBOT_SIZE / 2.0; // Distance from center to edge
    }

    public static class Intake {
        public static String intake = "intakeMotor"; //C 3
        public static String intakeBelt = "intakeBeltMotor"; // C 2

    }

    public static class Spindexer {
        public static SpindexerAndMotifStatus.SpindexerPattern spindexerPattern  = new SpindexerAndMotifStatus.SpindexerPattern(Enums.BallColor.None, Enums.BallColor.None, Enums.BallColor.None);
        public static String spindexerServo = "spindexerServo"; // E4
        public static String spindexerEncoder = "spindexerServoEncoder"; // E Analog 0-1

        public static String spindexerFLipperServo = "spindexerFlipperServo"; // E5
        //Constant Positions
        public final static double FLIPPER_POSITION_EXTENDED = 0.85;
        public final static double FLIPPER_POSITION_RETRACT = 0.51;

        public static final double FLICK_TIME = 0.15;

        // Spindexer PIDF coefficients - direction-specific for vertical mounting
        // CW rotation (with gravity assist)
        public static final double SPINDEXER_CW_P = 0.0066;
        public static final double SPINDEXER_CW_I = 0.0;
        public static final double SPINDEXER_CW_D = 0.0003;
        public static final double SPINDEXER_CW_F = 0.0001;

        // CCW rotation (against gravity)
        public static final double SPINDEXER_CCW_P = 0.0087;
        public static final double SPINDEXER_CCW_I = 0.0001;
        public static final double SPINDEXER_CCW_D = 0.0004;
        public static final double SPINDEXER_CCW_F = 0.0001;

        // Legacy default values (backwards compatibility)
        public static final double SPINDEXER_P = SPINDEXER_CCW_P;
        public static final double SPINDEXER_I = SPINDEXER_CCW_I;
        public static final double SPINDEXER_D = SPINDEXER_CCW_D;
        public static final double SPINDEXER_F = SPINDEXER_CCW_F;

        // Legacy PIDCoefficients for backwards compatibility
        public final static PIDCoefficients SPINDEXER_PID = new PIDCoefficients(SPINDEXER_P, SPINDEXER_I, SPINDEXER_D);

        // Acceptable position error (degrees) - stops rotation when within this range
        public static final double ANGLE_RANGE = 7;

        // Spindexer slot positions in degrees (3 equally-spaced slots: 120° apart)
        public static final int[] SPINDEXER_POSITIONS = {52, 172, 292};

    }

    public static class Cataloging {
        // Timeout for scanning state (no ball detected)
        public static final double SCAN_TIMEOUT_SECONDS = 2.0;

        // Timeout for rotation state (spindexer stuck)
        public static final double ROTATION_TIMEOUT_SECONDS = 3.0;

        // Number of retry attempts for stuck rotation
        public static final int MAX_ROTATION_RETRIES = 1;
    }

    public static class Shooter {
        //hardware
        public static String shooter1 = "shooterMotor1"; // Left(from back) shooter, C0
        public static String shooter2 = "shooterMotor2"; // Right(from back) shooter, C1
        public static String turret = "turretServo"; //E3
        public static String turretEncoder = "turretServoEncoder"; // Encoder port TBD
        public static String shooterFlipperServo = "shooterFlipperServo"; // C 0
        public static String shooterEncoder = "shooterEncoder";

        // 2600, 2100, is our powers

        //Constant Positions
        public final static double FLIPPER_POSITION_EXTENDED = 0.35;
        public final static double FLIPPER_POSITION_RETRACT = 0.15;
        public static final double FLICK_TIME = 0.15;

        // Shooter power settings

        // Turret offset from robot center (in inches, robot-relative)
        public static final double TURRET_OFFSET_X = 4;
        public static final double TURRET_OFFSET_Y = 1;

        // Turret positioning
        public static final double CENTER = 0.0;
        public static final double ANGLE_RANGE = 3.0; // Acceptable error in SERVO degrees (0.5° turret degrees)
        public static final double GEAR_RATIO = 6.0; // 6:1 servo to turret (servo rotates 6° for 1° turret rotation)

        // Turret tracking offset (in turret degrees) - compensates for systematic tracking error
        // Negative value shifts aim left, positive shifts aim right
        // Tune this if turret consistently misses left/right of target
        public static final double TURRET_TRACKING_OFFSET = 0; // Adjust if tracking is off

        // Turret PID coefficients (tuned values)
        public static final PIDCoefficients TURRET_PID = new PIDCoefficients(0.0035, .0015, 0.00030);

        // Shooter PIDF coefficients (tuned values from ShooterPIDFTuningTeleOp)
        public static final double SHOOTER_P = 40;
        public static final double SHOOTER_I = 0.0;
        public static final double SHOOTER_D = 0.0;
        public static final double SHOOTER_F = 10.0;
    }

    public static class ColorSensor {
        // Intake sensors (2 sensors offset to avoid ball holes at first spindexer slot)
        public static String intakeSensor1 = "intakeSensor1"; // First intake sensor
        public static String intakeSensor2 = "intakeSensor2"; // Second intake sensor (offset from first)

        // Color detection thresholds
        public static final double PURPLE_THRESHOLD = 700;
        public static final double GREEN_THRESHOLD = 1500;
        public static final double ALPHA_THRESHOLD = 500;

        // Detection logic: Green ball detected if (green > red) AND (green > GREEN_THRESHOLD)
    }

    public static class Controls {
        // Gamepad trigger activation threshold
        public static final double TRIGGER_THRESHOLD = 0.3;

        // Toggle to swap Red/Blue control mapping for testing
        // When true: Red uses standard controls, Blue uses inverted controls
        // When false (default): Blue uses standard controls, Red uses inverted controls
        public static final boolean SWAP_ALLIANCE_CONTROLS = true;
    }

    public static class Limelight{

        public static String limelight = "limelight";
        public static boolean isLimelightDisabled = false;
        public static SpindexerAndMotifStatus.MotifPattern motifPattern = new SpindexerAndMotifStatus.MotifPattern(Enums.BallColor.Purple, Enums.BallColor.Green, Enums.BallColor.Purple);

        // TAG_GOAL_POSITION for motif scanning (top center of field, slightly out of bounds)
        public static final double TAG_GOAL_X = 72.0; // inches
        public static final double TAG_GOAL_Y = 143.0; // inches

        // AprilTag to Motif mappings
        public static final Enums.BallColor[] APRILTAG_21_PATTERN = {
            Enums.BallColor.Green, Enums.BallColor.Purple, Enums.BallColor.Purple
        };
        public static final Enums.BallColor[] APRILTAG_22_PATTERN = {
            Enums.BallColor.Purple, Enums.BallColor.Green, Enums.BallColor.Purple
        };
        public static final Enums.BallColor[] APRILTAG_23_PATTERN = {
            Enums.BallColor.Purple, Enums.BallColor.Purple, Enums.BallColor.Green
        };
        public static boolean manuallySlowedForScan = true;

        public static Enums.BallColor[] getMotifPatternForTag(int tagId) {
            switch (tagId) {
                case 21: return APRILTAG_21_PATTERN;
                case 22: return APRILTAG_22_PATTERN;
                case 23: return APRILTAG_23_PATTERN;
                default: return null;
            }
        }
    }

    public static class Pinpoint{
        public static String pinpoint = "pinpoint"; //E I2C 1
        public static Pose2D standardStartPoint = new Pose2D(DistanceUnit.INCH,56.5, 8.5, AngleUnit.DEGREES, 90);

        // Yaw scalar for IMU drift correction
        // 1.0 = no correction (default starting point)
        // Tune this if heading drifts during rotation:
        // - Drive robot in 10 full rotations (3600 degrees)
        // - If Pinpoint reports 3580 degrees, set yawScalar = 3600/3580 = 1.0056
        // - If Pinpoint reports 3620 degrees, set yawScalar = 3600/3620 = 0.9945
        public static double yawScalar = 1.0;
    }
    public static class UpdatableConstants{
        public static double shooterVelocity;
        public static Pose endingAutonPose;

        public static Enums.AllianceColor allianceColor;
    }



    public static class Enums{
        public enum FieldState{
            IdleZone,
            ShootingZone,
            ParkZone,
            PenaltyZone
        }
        public enum AllianceColor {
            Red,
            Blue
        }
        public enum FlickState {
            Idle,
            Start,
            Extended,
            Retracted
        }
        public enum RotationState {
            IDLE,      // Ready for commands
            ROTATING   // Busy rotating
        }
        public enum ShooterCases{
            Idle,
            Start,
            SpindexerFlicking,
            ShooterFlicking,
            SpindexerRotating,
            BallShot

        }
        public enum BallColor{
            Purple,
            Green,
            None
        }
        public enum CatalogingCases{
            Idle,
            Scanning,
            WaitingForRotation,
            RetryRotation,
            RotateToEndLocation
        }
        public enum IntakeState {
            Idle,           // Motors stopped
            Intaking,       // Both motors forward (runs continuously)
            Reversing,      // Both motors backward (eject)
            StagingOnly,    // Only staging belt
            IntakeOnly      // Only intake motor
        }
        public enum DriveState {
            Idle,               // Not moving
            FieldRelative,      // Field-centric (default)
            RobotRelative,      // Robot-centric
            SlowMode,           // Precision mode
            AutoDriving,        // Autonomous navigation
            Locked              // Defense mode (X-pattern)
        }
        public enum LimelightMode {
            GoalTracking,       // Track basket for shooting
            TagTracking         // Scan for motif AprilTags
        }
    }
}
