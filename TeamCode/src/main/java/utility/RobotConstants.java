package utility;

import com.bylazar.configurables.annotations.Configurable;
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
@Configurable
public class RobotConstants {
    @Configurable
    public static class Drivetrain {
        // --- Hardware Names ---
        public static String frontLeftMotor = "frontLeftMotor"; //E0
        public static String backLeftMotor = "backLeftMotor"; // E1
        public static String frontRightMotor= "frontRightMotor"; // E2
        public static String backRightMotor = "backRightMotor"; // E3

        // --- Drive Tuning ---
        public static double STRAFE_COMPENSATION = 1.1;    // Counteract imperfect strafing
        public static double JOYSTICK_DEADBAND = 0.01;     // Minimum joystick input threshold
        public static double SLOW_MODE_MULTIPLIER = 0.25;  // Speed reduction for precision mode
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
    @Configurable
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
        public static double FLIPPER_POSITION_EXTENDED = 0.80;
        public static double FLIPPER_POSITION_RETRACT = 0.55;

        public static double FLICK_TIME = 0.02;

        // Spindexer PIDF coefficients - direction-specific for vertical mounting
        // CW rotation (with gravity assist)
        public static double SPINDEXER_CW_P = 0.0066;
        public static double SPINDEXER_CW_I = 0.0;
        public static double SPINDEXER_CW_D = 0.0003;
        public static double SPINDEXER_CW_F = 0.0001;

        // CCW rotation (against gravity)
        public static double SPINDEXER_CCW_P = 0.0087;
        public static double SPINDEXER_CCW_I = 0.0001;
        public static double SPINDEXER_CCW_D = 0.0004;
        public static double SPINDEXER_CCW_F = 0.0001;

        // Legacy default values (backwards compatibility)
        public static final double SPINDEXER_P = SPINDEXER_CCW_P;
        public static final double SPINDEXER_I = SPINDEXER_CCW_I;
        public static final double SPINDEXER_D = SPINDEXER_CCW_D;
        public static final double SPINDEXER_F = SPINDEXER_CCW_F;

        // Legacy PIDCoefficients for backwards compatibility
        public static PIDCoefficients SPINDEXER_PID = new PIDCoefficients(SPINDEXER_P, SPINDEXER_I, SPINDEXER_D);

        // Acceptable position error (degrees) - stops rotation when within this range
        public static double ANGLE_RANGE = 7;

        // Spindexer slot positions in degrees (3 equally-spaced slots: 120° apart)
        public static int[] SPINDEXER_POSITIONS = {52, 172, 292};

        // --- Position Detection Thresholds ---
        public static double ANGLE_WITHIN_RANGE_THRESHOLD = 60.0;  // Degrees for "within slot range" check
        public static double ENCODER_READY_THRESHOLD = 0.01;       // Voltage change to detect encoder ready

        // --- Tuning ---
        public static double TUNING_TARGET_POSITION = 52.0;  // Target position for PIDF tuning (degrees)
    }

    @Configurable
    public static class Cataloging {
        // Timeout for scanning state (no ball detected)
        public static double SCAN_TIMEOUT_SECONDS = 2.0;

        // Timeout for rotation state (spindexer stuck)
        public static double ROTATION_TIMEOUT_SECONDS = 3.0;

        // Number of retry attempts for stuck rotation
        public static int MAX_ROTATION_RETRIES = 1;
    }
    @Configurable
    public static class Shooter {
        // ==================== HARDWARE NAMES ====================
        public static String shooter1 = "shooterMotor1";      // Left flywheel (from back), C0
        public static String shooter2 = "shooterMotor2";      // Right flywheel (from back), C1
        public static String turret = "turretServo";          // Continuous rotation servo, E3
        public static String turretEncoder = "turretServoEncoder"; // Analog encoder for turret position
        public static String shooterFlipperServo = "shooterFlipperServo"; // Ball flipper servo, C0
        public static String shooterEncoder = "shooterEncoder";

        // ==================== FLIPPER POSITIONS ====================
        public static double FLIPPER_POSITION_EXTENDED = 0.35;  // Push ball into flywheel
        public static double FLIPPER_POSITION_RETRACT = 0.15;   // Ready position
        public static double FLICK_TIME = 0.15;                 // Seconds to hold extended

        // ==================== GOAL POSITIONS ====================
        // Goal positions in FIELD/PINPOINT coordinates (inches)
        // Blue goal: top-left corner of field
        // Red goal: top-right corner of field
        public static double BLUE_GOAL_X = 0.0;
        public static double BLUE_GOAL_Y = 141.0;
        public static double RED_GOAL_X = 136.0;
        public static double RED_GOAL_Y = 141.0;

        // ==================== TURRET GEOMETRY ====================
        // Turret offset from robot center (inches, robot-relative frame)
        // When robot faces forward (heading=90°):
        //   - OFFSET_X = 4" to the RIGHT of center
        //   - OFFSET_Y = 1" FORWARD of center
        // These are transformed to field coordinates using the robot heading
        public static double TURRET_OFFSET_X = 4.0;
        public static double TURRET_OFFSET_Y = 1.0;

        // ==================== TURRET LIMITS ====================
        // Turret physical limits (turret degrees, not servo degrees)
        // Negative = left (CCW), Positive = right (CW)
        // Range: -45° to +60° turret degrees (×5 gear ratio = -225° to +300° servo degrees)
        public static double TURRET_MIN_ANGLE = -45.0;  // Max left rotation (CCW)
        public static double TURRET_MAX_ANGLE = 60.0;   // Max right rotation (CW)

        // ==================== TURRET CONTROL ====================
        // Target angle when not tracking (turret degrees)
        public static double CENTER = 0.0;

        // PID deadband: stops motor when error < ANGLE_RANGE (servo degrees)
        // 1.5° servo = 0.3° turret (due to 5:1 gear ratio)
        public static double ANGLE_RANGE = 1.5;

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
        public static final double GEAR_RATIO = 5.0;

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
        public static double TURRET_TRACKING_OFFSET = 0;

        // Turret PID gains (operates in servo degrees)
        // P=0.0035: Low gain for smooth tracking (may need increase if slow)
        // I=0.0015: Small integral for steady-state error
        // D=0.00030: Damping to prevent overshoot
        public static PIDCoefficients TURRET_PID = new PIDCoefficients(0.0035, 0.0015, 0.00030);

        // Target angle for turret tuning mode (turret degrees, not servo degrees)
        // Adjust this via configurables to test turret PID at different positions
        public static double TURRET_TUNING_TARGET = 0.0;

        // ==================== FLYWHEEL PIDF ====================
        // Velocity control for flywheel motors (ticks/sec)
        public static double SHOOTER_P = 40;
        public static double SHOOTER_I = 0.0;
        public static double SHOOTER_D = 0.0;
        public static double SHOOTER_F = 10.2;

        // Target velocity for shooter tuning mode (ticks/sec)
        // Adjust this via configurables to test shooter PIDF at different speeds
        public static double SHOOTER_TUNING_VELOCITY = 2200.0;

        // ==================== VELOCITY CONSTANTS ====================
        public static double DEFAULT_VELOCITY = 2200.0;           // Default/initial flywheel velocity
        public static double FALLBACK_VELOCITY = 2200.0;          // Fallback when lookup fails

        // ==================== VELOCITY LOOKUP TABLES ====================
        // Zone boundary (inches) - below uses front table, above uses back table
        public static double BLUE_ZONE_BOUNDARY = 105.0;
        public static double RED_ZONE_BOUNDARY = 100.0;

        // Blue Front Zone: distance (inches) to velocity (ticks/sec)
        public static double[][] BLUE_FRONT_LOOKUP = {
            {53, 1900},
            {78, 2000},
            {80, 1950},
            {94, 2030},
            {98, 2100},
            {100, 2100}
        };

        // Blue Back Zone
        public static double[][] BLUE_BACK_LOOKUP = {
            {140, 2600},
            {144, 2600},
            {148, 2650},
            {154, 2700}
        };

        // Red Front Zone
        public static double[][] RED_FRONT_LOOKUP = {
            {48, 2000},
            {52, 2000},
            {80, 2100},
            {85, 2100}
        };

        // Red Back Zone
        public static double[][] RED_BACK_LOOKUP = {
            {134, 2500},
            {138, 2520},
            {148, 2650}
        };

        // ==================== TURRET CONTROL THRESHOLDS ====================
        public static double TURRET_TARGET_CHANGE_THRESHOLD = 0.5;  // Min degrees change to reset PID
        public static double TURRET_DRIFT_THRESHOLD = 30.0;         // Degrees of drift before flagging
        public static double TURRET_LIMIT_MARGIN = 30.0;            // Safety margin before hardware limits
        public static double TURRET_POWER_LIMIT_NORMAL = 0.5;       // Normal max power
        public static double TURRET_POWER_LIMIT_NEAR_EDGE = 0.3;    // Max power near hardware limits

    }

    /**
     * Shooter PIDF values in a separate class for independent live tuning via Panels.
     * Refreshing this class won't affect turret PID or other Shooter settings.
     */
    @Configurable
    public static class ShooterPIDF {
        public static double P = 16;
        public static double I = 0.0;
        public static double D = 0.0;
        public static double F = 10;
        public static double TUNING_VELOCITY = 2625.0;
    }

    @Configurable
    public static class Encoder {
        // --- Analog Encoder Conversion ---
        public static double MAX_VOLTAGE = 3.3;                    // Max voltage from analog encoder
        public static double FULL_ROTATION_DEGREES = 360.0;        // Full rotation in degrees

        // --- Angle Normalization Bounds ---
        public static double ANGLE_UPPER_BOUND = 180.0;            // Upper bound for normalization
        public static double ANGLE_LOWER_BOUND = -180.0;           // Lower bound for normalization
    }

    @Configurable
    public static class PID {
        // --- Delta Time Validation ---
        public static double DT_MAX = 1.0;                         // Max delta time (skip if exceeded)
        public static double DT_MIN = 0.001;                       // Min delta time (skip if below)
        public static double DT_DEFAULT = 0.02;                    // Default delta time (50Hz)

        // --- Anti-Windup ---
        public static double INTEGRAL_CLAMP_MAX = 50.0;            // Upper integral windup limit
        public static double INTEGRAL_CLAMP_MIN = -50.0;           // Lower integral windup limit

        // --- Tuning Step Sizes ---
        public static double[] TUNING_STEP_SIZES = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    }

    @Configurable
    public static class ShootingSequence {
        // Total number of balls in the shooting sequence
        public static int TOTAL_BALLS = 3;

        // Wait time after spindexer flipper activates before removing ball from slot (seconds)
        public static double SHOOTER_FLIPPER_TIME = 0.02;

        // Extra wait time after starting rotation before firing shooter flipper (seconds)
        public static double EXTRA_WAIT_TIME = 0.075;

        // Time spindexer must be within tolerance before declaring rotation complete (seconds)
        public static double SPINDEXER_SETTLING_TIME = 0;
    }

    public static class ColorSensor {
        // Intake sensors (2 sensors offset to avoid ball holes at first spindexer slot)
        public static String intakeSensor1 = "intakeSensor1"; // First intake sensor
        public static String intakeSensor2 = "intakeSensor2"; // Second intake sensor (offset from first)
    }

    public static class Controls {
        // Gamepad trigger activation threshold
        public static final double TRIGGER_THRESHOLD = 0.3;

        // Toggle to swap Red/Blue control mapping for testing
        // When true: Red uses standard controls, Blue uses inverted controls
        // When false (default): Blue uses standard controls, Red uses inverted controls
        public static boolean SWAP_ALLIANCE_CONTROLS = true;
    }
    @Configurable
    public static class Limelight {
        public static String limelight = "limelight";
        public static boolean isLimelightDisabled = false;

        // Current motif pattern being matched
        public static SpindexerAndMotifStatus.MotifPattern motifPattern =
            new SpindexerAndMotifStatus.MotifPattern(Enums.BallColor.Purple, Enums.BallColor.Green, Enums.BallColor.Purple);

        // AprilTag scanning target position (field coordinates)
        // Top center of field where motif tags are located
        // X=72 = center, Y=143 = top edge (near goals)
        public static double TAG_GOAL_X = 72.0;  // inches, field center
        public static double TAG_GOAL_Y = 143.0; // inches, top of field

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
    @Configurable
    public static class Pinpoint{
        public static String pinpoint = "pinpoint"; //E I2C 3

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
        public static double yawScalar = .998148;
    }
    public static class UpdatableConstants{
        public static double shooterVelocity;
        public static Pose endingAutonPose;

        public static Enums.AllianceColor allianceColor;
    }

    public static class Park {
        // Blue park zone - adjust coordinates for your field
        public static Pose redParkZone = new Pose(38.75, 32.5, Math.toRadians(90));

        // Red park zone - adjust coordinates for your field
        public static Pose blueParkZone = new Pose(105.25, 32.5, Math.toRadians(90));
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
            Parking,            // Auto park with position hold
            Locked              // Defense mode (X-pattern)
        }
        public enum LimelightMode {
            GoalTracking,       // Track basket for shooting
            TagTracking         // Scan for motif AprilTags
        }
    }
}
