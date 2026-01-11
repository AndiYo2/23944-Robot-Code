package Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
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

    // --- Spindexer PIDF coefficients - direction-specific for vertical mounting ---
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

    // Acceptable position error (degrees) - stops rotation when within this range
    public static double ANGLE_RANGE = 7;

    // Spindexer slot positions in degrees (3 equally-spaced slots: 120 apart)
    public static int[] SPINDEXER_POSITIONS = {52, 172, 292};

    // --- Position Detection Thresholds ---
    public static double ANGLE_WITHIN_RANGE_THRESHOLD = 60.0;  // Degrees for "within slot range" check
    public static double ENCODER_READY_THRESHOLD = 0.01;       // Voltage change to detect encoder ready

    // --- Tuning ---
    public static double TUNING_TARGET_POSITION = 52.0;  // Target position for PIDF tuning (degrees)
}
