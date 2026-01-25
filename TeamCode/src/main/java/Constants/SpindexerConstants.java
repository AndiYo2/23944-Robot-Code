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
    public static double FLIPPER_POSITION_EXTENDED = 0.85;
    public static double FLIPPER_POSITION_RETRACT = 0.55;
    public static double FLICK_TIME = 0.075;

    // --- Degree-Based Position Constants ---
    // All positions in SERVO degrees (0-300° effective range)
    // Conversion to servo position: degrees / 355.0
    //
    // 6 positions for 3 slots (2 positions per slot due to 2:1 gear ratio):
    //   Slot 0: 0°, 180°
    //   Slot 1: 60°, 240°
    //   Slot 2: 120°, 300°
    public static final int DEGREE_INCREMENT = 60;          // One slot = 60°
    public static final int MAX_SERVO_DEGREES = 300;        // Max usable range
    public static final int[] SLOT_POSITIONS_DEG = {0, 60, 120, 180, 240, 300};
    public static final int SLOTS_COUNT = 3;

    // Boundary wrapping (in degrees) - safety net, should rarely be used
    public static final int CW_WRAP_TO_DEG = 180;           // At 300°, CW wraps to 180°
    public static final int CCW_WRAP_TO_DEG = 120;          // At 0°, CCW wraps to 120°

    // Empty/reset position
    public static final int EMPTY_RESET_DEGREES = 300;

    // Servo conversion constant
    public static final double SERVO_DEGREES_PER_UNIT = 355.0;  // Axon servo full range

    // Offset applied to ALL spindexer rotations (in degrees)
    public static double SPINDEXER_OFFSET = 0;

    // Cataloging timing
    public static double ROTATION_TIME = 0.15;
    public static double INTAKE_TIME = 0.1;

    // Shooting mode (Fast or Sorted) - can be set by auto
    public static EnumConstants.ShootingMode currentMode = EnumConstants.ShootingMode.Sorted;

    // Shooting and cataloging timing constants

    public static double SHOOTER_FLIPPER_OUT_TIME = 0.075;

    public static double SPINDEXER_FLIPPER_OUT_TIME = 0.075; // Wait after spindexer flipper (same as FLICK_TIME)
    public static double INTAKE_TIMING = 0.35;                // Wait during intake operation
}
