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
    public static double SHOOTER_FLIPPER_EXTENDED = 0.28;//.26
    public static double SHOOTER_FLIPPER_RETRACT = 0.48;
    public static double SHOOTER_FLICK_TIME = 0.15;//.12
    public static double SHOOTER_RETRACT_DELAY = 0.125;//.1

    // --- Spindexer Flipper ---
    public static double SPINDEXER_FLIPPER_EXTENDED = 0.42;
    public static double SPINDEXER_FLIPPER_RETRACT = 0.502;
    public static double SPINDEXER_FLICK_TIME = 0.075;
    public static double SPINDEXER_RETRACT_DELAY = 0;

    // --- Pipelining ---
    public static double SHOOTER_EXTEND_HALFWAY = 0.06; // 0.03

    // --- Spindexer Rotation ---
    public static double SPINDEXER_ROTATION_TIME = 0.16;//0.15
}