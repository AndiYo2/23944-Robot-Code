package Constants;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Drivetrain tuning constants.
 */
@Configurable
public class DriveConstants {
    // --- Drive Tuning ---
    public static double STRAFE_COMPENSATION = 1.1;    // Counteract imperfect strafing
    public static double JOYSTICK_DEADBAND = 0.01;     // Minimum joystick input threshold
    public static double SLOW_MODE_MULTIPLIER = 0.25;  // Speed reduction for precision mode
}
