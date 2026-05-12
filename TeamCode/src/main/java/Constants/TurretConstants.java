package Constants;

import com.bylazar.configurables.annotations.Configurable;


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
    public static final double GEAR_RATIO = 200.0 / 96.0;

    // These are what its supposed to be, but I lowkey think they are flipped
    public static double HARD_STOP_CW = 55.0;
    public static double HARD_STOP_CCW = -70.0;
    public static double CENTER = 1.5;
    public static double SMOOTHING_ALPHA = 0.8;

    public static double BLUE_BACK_TURRET_TRACKING_OFFSET = 0.5;
    public static double BLUE_FRONT_TURRET_TRACKING_OFFSET = 0.5;

    public static double RED_BACK_TURRET_TRACKING_OFFSET = 2;
    public static double RED_FRONT_TURRET_TRACKING_OFFSET = -1;
    public static double TURRET_OFFSET_Y_THRESHOLD = 48.0;
}