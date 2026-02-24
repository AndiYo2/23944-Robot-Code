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

    public static final double SERVO_CENTER_POSITION = 0.5;
    public static final double SERVO_DEGREES_PER_UNIT = SpindexerConstants.SERVO_DEGREES_PER_UNIT;
    // Servo-to-turret gear ratio (2.5:1)
    public static final double GEAR_RATIO = 200/96;

    // ==================== TURRET LIMITS ====================
    // Hard stop limits (turret degrees) - prevents hardware damage
    // Negative = left (CCW), Positive = right (CW)
    public static double HARD_STOP_CW = 60.0;    // Max right rotation (clockwise)
    public static double HARD_STOP_CCW = -70.0;  // Max left rotation (counter-clockwise)
    public static double CENTER = 0;
    public static double MIN_CHANGE_THRESHOLD = 0.5;
    /** EMA smoothing factor for turret angle (0.0 = frozen, 1.0 = no smoothing). */
    public static double SMOOTHING_ALPHA = 0.35;

    // Positive = shift aim right, Negative = shift aim left
    public static double BLUE_TURRET_TRACKING_OFFSET = 2;
    public static double RED_TURRET_TRACKING_OFFSET = -2;
}
