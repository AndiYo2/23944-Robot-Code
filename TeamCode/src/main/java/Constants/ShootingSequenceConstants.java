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
    public static double SHOOTER_FLIPPER_EXTENDED = 0.3;
    public static double SHOOTER_FLIPPER_RETRACT = 0.4675;
    public static double SHOOTER_FLICK_TIME = 0.15;//.13
    public static double SHOOTER_RETRACT_DELAY = 0.09;//.06

    // --- Spindexer Flipper ---
    public static double SPINDEXER_FLIPPER_EXTENDED = 0.41;
    public static double SPINDEXER_FLIPPER_RETRACT = 0.506;
    public static double SPINDEXER_FLICK_TIME = 0.09;
    public static double SPINDEXER_RETRACT_DELAY = 0.06;//.02

    // --- Pipelining ---
    public static double SHOOTER_EXTEND_HALFWAY = 0.015;

    // --- Spindexer Rotation ---
    public static double SPINDEXER_ROTATION_TIME = 0.15;//0.14
}