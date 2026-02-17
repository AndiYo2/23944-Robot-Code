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
    public static double SHOOTER_FLIPPER_EXTENDED = 0.25;
    public static double SHOOTER_FLIPPER_RETRACT = 0.46;
    public static double SHOOTER_FLICK_TIME = 0.09;
    public static double SHOOTER_RETRACT_DELAY = 0.075;

    // --- Spindexer Flipper ---
    public static double SPINDEXER_FLIPPER_EXTENDED = 0.4;
    public static double SPINDEXER_FLIPPER_RETRACT = 0.55;
    public static double SPINDEXER_FLICK_TIME = 0.075;
    public static double SPINDEXER_RETRACT_DELAY = 0.025;

    // --- Spindexer Rotation ---
    public static double SPINDEXER_ROTATION_TIME = 0.15;
}