package Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

/**
 * Turret subsystem constants.
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

    // ==================== TURRET LIMITS ====================
    // Turret physical limits (turret degrees, not servo degrees)
    // Negative = left (CCW), Positive = right (CW)
    // Range: -45 to +60 turret degrees (x5 gear ratio = -225 to +300 servo degrees)
    public static double TURRET_MIN_ANGLE = -45.0;  // Max left rotation (CCW)
    public static double TURRET_MAX_ANGLE = 60.0;   // Max right rotation (CW)

    // ==================== TURRET CONTROL ====================
    // Target angle when not tracking (turret degrees)
    public static double CENTER = 0.0;

    // PID deadband: stops motor when error < ANGLE_RANGE (servo degrees)
    // 1.5 servo = 0.3 turret (due to 5:1 gear ratio)
    public static double ANGLE_RANGE = 1.5;

    // Servo-to-turret gear ratio (5:1)
    // Servo rotates GEAR_RATIO degrees for every 1 of turret rotation
    // All PID math uses SERVO degrees; divide by GEAR_RATIO for turret degrees
    //
    // HOW TO VERIFY/TUNE:
    //   1. Center turret (0), note encoder reading
    //   2. Manually rotate turret exactly 30 (use protractor)
    //   3. Note new encoder reading
    //   4. GEAR_RATIO = (encoder_change) / 30
    //   Example: If encoder changes by 150 for 30 turret rotation, GEAR_RATIO = 150/30 = 5.0
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
    // "turret centered" = "0 encoder position"
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

    // ==================== TURRET CONTROL THRESHOLDS ====================
    public static double TURRET_TARGET_CHANGE_THRESHOLD = 0.5;  // Min degrees change to reset PID
    public static double TURRET_DRIFT_THRESHOLD = 30.0;         // Degrees of drift before flagging
    public static double TURRET_LIMIT_MARGIN = 30.0;            // Safety margin before hardware limits
    public static double TURRET_POWER_LIMIT_NORMAL = 0.5;       // Normal max power
    public static double TURRET_POWER_LIMIT_NEAR_EDGE = 0.3;    // Max power near hardware limits
}
