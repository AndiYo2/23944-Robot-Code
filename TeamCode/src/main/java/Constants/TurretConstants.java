package Constants;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Turret subsystem constants.
 *
 * The turret uses a position-controlled servo (not continuous rotation).
 * Servo position 0.5 = turret center (0 degrees)
 * Gear ratio is 2.5:1 (servo rotates 2.5x per turret degree)
 */
@Configurable
public class TurretConstants {
    // ==================== TURRET GEOMETRY ====================
    // Turret offset from robot center (inches, robot-relative frame)
    // When robot faces forward (heading=90):
    //   - OFFSET_X = 4" to the RIGHT of center
    //   - OFFSET_Y = 1" FORWARD of center
    // These are transformed to field coordinates using the robot heading
    public static double TURRET_OFFSET_X = 4.0;
    public static double TURRET_OFFSET_Y = 1.0;

    // ==================== SERVO CONFIGURATION ====================
    // Servo range: 0-355 degrees (position 0 = 0°, position 1 = 355°)
    // Center position (0° turret) is at servo position 0.5 (~177.5° servo)
    public static final double SERVO_CENTER_POSITION = 0.5;
    // Reference SpindexerConstants for Axon servo full range (single source of truth)
    public static final double SERVO_DEGREES_PER_UNIT = SpindexerConstants.SERVO_DEGREES_PER_UNIT;
    public static final double MIN_SERVO_POSITION = 0.0001;  // Never set to exactly 0

    // Servo-to-turret gear ratio (2.5:1)
    // Servo rotates GEAR_RATIO degrees for every 1 degree of turret rotation
    public static final double GEAR_RATIO = 2.5;

    // ==================== TURRET LIMITS ====================
    // Hard stop limits (turret degrees) - prevents hardware damage
    // Negative = left (CCW), Positive = right (CW)
    public static double HARD_STOP_CW = 60.0;    // Max right rotation (clockwise)
    public static double HARD_STOP_CCW = -45.0;  // Max left rotation (counter-clockwise)

    // ==================== TURRET CONTROL ====================
    // Target angle when not tracking (turret degrees)
    public static double CENTER = 0.0;

    // Minimum change threshold (turret degrees)
    // Only update servo if the change exceeds this value (prevents noise/jitter)
    public static double MIN_CHANGE_THRESHOLD = 0.5;

    // TURRET_TRACKING_OFFSET: Fine-tune adjustment for systematic aim error (turret degrees)
    // Use this for small adjustments to correct aim
    // Positive = shift aim right, Negative = shift aim left
    public static double TURRET_TRACKING_OFFSET = 0;

    // Target angle for turret tuning mode (turret degrees)
    // Adjust this via configurables to test turret at different positions
    public static double TURRET_TUNING_TARGET = 0.0;
}
