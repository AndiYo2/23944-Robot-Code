package utility;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Robot configuration constants and hardware mappings.
 *
 * COORDINATE SYSTEM (Pedro Pathing / Pinpoint):
 *   - Origin (0,0) at bottom-left of field
 *   - +X = RIGHT (increases toward Red alliance side)
 *   - +Y = FORWARD/UP (increases toward goals)
 *   - Heading: 0° = facing right (+X), 90° = facing forward (+Y)
 *   - Rotation: Counter-clockwise is positive
 *   - Field size: 144" x 144"
 *
 * ROBOT-RELATIVE FRAME:
 *   - Forward = +Y direction when heading = 90°
 *   - Right = +X direction when heading = 90°
 *   - Turret offsets use robot-relative convention
 *
 * HARDWARE PORT COMMENTS:
 *   - E = Expansion Hub
 *   - C = Control Hub
 *   - Numbers indicate port indices
 */
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

        // Default preload pattern for autonomous (skips cataloging)
        // Slot 0 = Intake, Slot 1 = Shooter, Slot 2 = Top Storage
        public static final Enums.BallColor[] DEFAULT_PRELOAD = {
            Enums.BallColor.Purple,  // Intake
            Enums.BallColor.Purple,  // Shooter
            Enums.BallColor.Green    // Top Storage
        };
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
        // ==================== HARDWARE NAMES ====================
        public static String shooter1 = "shooterMotor1";      // Left flywheel (from back), C0
        public static String shooter2 = "shooterMotor2";      // Right flywheel (from back), C1
        public static String turret = "turretServo";          // Continuous rotation servo, E3
        public static String turretEncoder = "turretServoEncoder"; // Analog encoder for turret position
        public static String shooterFlipperServo = "shooterFlipperServo"; // Ball flipper servo, C0
        public static String shooterEncoder = "shooterEncoder";

        // ==================== FLIPPER POSITIONS ====================
        public final static double FLIPPER_POSITION_EXTENDED = 0.35;  // Push ball into flywheel
        public final static double FLIPPER_POSITION_RETRACT = 0.15;   // Ready position
        public static final double FLICK_TIME = 0.15;                 // Seconds to hold extended

        // ==================== TURRET GEOMETRY ====================
        // Turret offset from robot center (inches, robot-relative frame)
        // When robot faces forward (heading=90°):
        //   - OFFSET_X = 4" to the RIGHT of center
        //   - OFFSET_Y = 1" FORWARD of center
        // These are transformed to field coordinates using the robot heading
        public static final double TURRET_OFFSET_X = 4.0;
        public static final double TURRET_OFFSET_Y = 1.0;

        // ==================== TURRET LIMITS ====================
        // Turret physical limits (turret degrees, not servo degrees)
        // Negative = left (CCW), Positive = right (CW)
        // Range: -45° to +60° turret degrees (×5 gear ratio = -225° to +300° servo degrees)
        public static final double TURRET_MIN_ANGLE = -45.0;  // Max left rotation (CCW)
        public static final double TURRET_MAX_ANGLE = 60.0;   // Max right rotation (CW)

        // ==================== TURRET CONTROL ====================
        // Target angle when not tracking (turret degrees)
        public static final double CENTER = 0.0;

        // PID deadband: stops motor when error < ANGLE_RANGE (servo degrees)
        // 1.5° servo = 0.3° turret (due to 5:1 gear ratio)
        public static final double ANGLE_RANGE = 1.5;

        // Servo-to-turret gear ratio (5:1)
        // Servo rotates GEAR_RATIO degrees for every 1° of turret rotation
        // All PID math uses SERVO degrees; divide by GEAR_RATIO for turret degrees
        //
        // HOW TO VERIFY/TUNE:
        //   1. Center turret (0°), note encoder reading
        //   2. Manually rotate turret exactly 30° (use protractor)
        //   3. Note new encoder reading
        //   4. GEAR_RATIO = (encoder_change) / 30
        //   Example: If encoder changes by 150° for 30° turret rotation, GEAR_RATIO = 150/30 = 5.0
        public static double GEAR_RATIO = 5.0;

        // ==================== ENCODER CALIBRATION ====================
        // TURRET_ENCODER_OFFSET: Encoder reading (in degrees) when turret is physically centered
        //
        // HOW TO CALIBRATE:
        //   1. Manually center the turret so it points straight forward
        //   2. Run the TurretCenteringTool or read encoder voltage
        //   3. Calculate: offset = (voltage / 3.3) * 360.0
        //   4. Set TURRET_ENCODER_OFFSET to that value
        //
        // This offset is SUBTRACTED from raw encoder reading so that
        // "turret centered" = "0° encoder position"
        //
        // CRITICAL: If this is wrong, ALL turret angles will be off by a constant amount!
        public static double TURRET_ENCODER_OFFSET = 0.0;  

        // TURRET_TRACKING_OFFSET: Fine-tune adjustment for systematic aim error (turret degrees)
        // Use this for small adjustments AFTER encoder is calibrated
        // Positive = shift aim right, Negative = shift aim left
        public static final double TURRET_TRACKING_OFFSET = 0;

        // Turret PID gains (operates in servo degrees)
        // P=0.0035: Low gain for smooth tracking (may need increase if slow)
        // I=0.0015: Small integral for steady-state error
        // D=0.00030: Damping to prevent overshoot
        public static final PIDCoefficients TURRET_PID = new PIDCoefficients(0.0035, 0.0015, 0.00030);

        // ==================== FLYWHEEL PIDF ====================
        // Velocity control for flywheel motors (ticks/sec)
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

    public static class Limelight {
        public static String limelight = "limelight";
        public static boolean isLimelightDisabled = false;

        // Current motif pattern being matched
        public static SpindexerAndMotifStatus.MotifPattern motifPattern =
            new SpindexerAndMotifStatus.MotifPattern(Enums.BallColor.Purple, Enums.BallColor.Green, Enums.BallColor.Purple);

        // AprilTag scanning target position (field coordinates)
        // Top center of field where motif tags are located
        // X=72 = center, Y=143 = top edge (near goals)
        public static final double TAG_GOAL_X = 72.0;  // inches, field center
        public static final double TAG_GOAL_Y = 143.0; // inches, top of field

        // AprilTag ID to ball pattern mappings
        // Each tag indicates which color ball should be in each spindexer slot
        public static final Enums.BallColor[] APRILTAG_21_PATTERN = {
            Enums.BallColor.Green, Enums.BallColor.Purple, Enums.BallColor.Purple
        };
        public static final Enums.BallColor[] APRILTAG_22_PATTERN = {
            Enums.BallColor.Purple, Enums.BallColor.Green, Enums.BallColor.Purple
        };
        public static final Enums.BallColor[] APRILTAG_23_PATTERN = {
            Enums.BallColor.Purple, Enums.BallColor.Purple, Enums.BallColor.Green
        };

        // Slows flywheel during tag scanning for camera stability
        public static boolean manuallySlowedForScan = true;

        /** Returns the ball pattern for a given AprilTag ID, or null if unknown. */
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

        // Alliance-specific start positions (robot facing forward at Y=8.5, heading 90°)
        // Blue Alliance: left side of field (X=56.5)
        // Red Alliance: right side of field (X=87.5)
        public static Pose2D blueStartPoint = new Pose2D(DistanceUnit.INCH, 56.5, 8.5, AngleUnit.DEGREES, 90);
        public static Pose2D redStartPoint = new Pose2D(DistanceUnit.INCH, 87.5, 8.5, AngleUnit.DEGREES, 90);

        // Default start point (Blue alliance by default)
        public static Pose2D standardStartPoint = blueStartPoint;

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
