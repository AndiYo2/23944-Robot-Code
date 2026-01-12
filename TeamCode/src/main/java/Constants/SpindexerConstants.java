package Constants;

import com.bylazar.configurables.annotations.Configurable;
import utility.SpindexerAndMotifStatus;

/**
 * Spindexer subsystem constants.
 */
@Configurable
public class SpindexerConstants {
    public static SpindexerAndMotifStatus.SpindexerPattern spindexerPattern =
        new SpindexerAndMotifStatus.SpindexerPattern(EnumConstants.BallColor.None, EnumConstants.BallColor.None, EnumConstants.BallColor.None);

    // Default preload pattern for autonomous (skips cataloging)
    // Slot 0 = Intake, Slot 1 = Shooter, Slot 2 = Top Storage
    public static final EnumConstants.BallColor[] DEFAULT_PRELOAD = {
        EnumConstants.BallColor.Purple,  // Intake
        EnumConstants.BallColor.Purple,  // Shooter
        EnumConstants.BallColor.Green    // Top Storage
    };

    // --- Flipper Positions ---
    public static double FLIPPER_POSITION_EXTENDED = 0.80;
    public static double FLIPPER_POSITION_RETRACT = 0.55;
    public static double FLICK_TIME = 0.02;

    // --- Servo Position Constants (Axon servo in position mode) ---
    // 6 positions for 3 slots (2 positions per slot due to 2:1 gear ratio)
    public static final double POSITION_INCREMENT = 0.2;
    public static final double[] SERVO_POSITIONS = {0.0, 0.2, 0.4, 0.6, 0.8, 1.0};
    public static final int SLOTS_COUNT = 3;

    // Boundary wrapping (safety net - should rarely be used in normal operation)
    public static final double CW_WRAP_TO = 0.6;   // At 1.0, CW wraps to 0.6
    public static final double CCW_WRAP_TO = 0.4;  // At 0.0, CCW wraps to 0.4

    // Empty/reset position
    public static final double EMPTY_RESET_POSITION = 0.0;

    // --- Timing Constants ---
    public static final double ROTATION_TIME_MS = 150;       // Normal 0.2 position change
    public static final double WRAP_ROTATION_TIME_MS = 300;  // Wrap 0.4 position change
    public static final double VERIFICATION_TIMEOUT_MS = 50; // Time to verify encoder position

    // --- Encoder Verification ---
    public static final double ENCODER_TOLERANCE_DEG = 15.0;  // Acceptable error in degrees
    public static final int MAX_RETRY_ATTEMPTS = 2;           // Auto-retry on verification failure

    // --- Position Detection Thresholds ---
    public static double ANGLE_WITHIN_RANGE_THRESHOLD = 60.0;  // Degrees for "within slot range" check
    public static double ENCODER_READY_THRESHOLD = 0.01;       // Voltage change to detect encoder ready

    // Spindexer slot positions in degrees (for encoder verification)
    // With 2:1 ratio: servo 0-300° = spindexer 0-600°
    // Slot 0: 0°, 360° (servo 0.0, 0.6)
    // Slot 1: 120°, 480° (servo 0.2, 0.8)
    // Slot 2: 240°, 600° (servo 0.4, 1.0)
    public static final double[] SLOT_ENCODER_POSITIONS_DEG = {0, 120, 240};
}
