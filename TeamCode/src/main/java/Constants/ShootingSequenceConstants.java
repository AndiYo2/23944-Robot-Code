package Constants;

import com.bylazar.configurables.annotations.Configurable;

/**
 * All timing and position constants for the shooting sequence.
 * Covers both the shooter flipper and spindexer flipper state machines,
 * plus spindexer rotation timing.
 */
@Configurable
public class ShootingSequenceConstants {
    // --- Shooter Flipper ---
    public static double SHOOTER_FLIPPER_EXTENDED = 0.44;
    public static double SHOOTER_FLIPPER_RETRACT = 0.514;
    public static double SHOOTER_FLICK_TIME = 0.09;
    public static double SHOOTER_RETRACT_DELAY = 0;

    // --- Spindexer Flipper ---
    public static double SPINDEXER_FLIPPER_EXTENDED = 0.4205;
    public static double SPINDEXER_FLIPPER_RETRACT = 0.5175;
    public static double SPINDEXER_FLICK_TIME = 0.1;
    public static double SPINDEXER_RETRACT_DELAY = 0.02;

    // --- Equalization delay (seconds) before Ball 2 to match Ball 2→3 interval ---
    public static double SHOT_EQUALIZATION_DELAY = 0.04;

    // --- Slow Shoot Delays (seconds between each ball) ---
    public static double SLOW_SHOOT_DELAY = 0.25;
    public static double SUPER_SLOW_SHOOT_DELAY = 0.5;

    // --- Pipelining ---
    public static double SHOOTER_EXTEND_HALFWAY = 0;

    // --- Spindexer Rotation ---
    public static double SPINDEXER_ROTATION_TIME = 0.11;

    // --- Analog Encoder Tolerances ---
    public static double SHOOTER_FLIPPER_POSITION_TOLERANCE = 0.02;
    public static double SPINDEXER_FLIPPER_POSITION_TOLERANCE = 0.02;
}
