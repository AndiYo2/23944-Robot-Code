package Constants;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Drivetrain tuning constants.
 */
@Configurable
public class DriveConstants {
    public static double STRAFE_COMPENSATION = 1.1;
    public static double JOYSTICK_DEADBAND = 0.01;
    public static double SLOW_MODE_MULTIPLIER = 0.25;
    public static double DYNAMIC_SLOW_DEADBAND = 0.15;
    public static double DYNAMIC_SLOW_MIN = 0.25;
}
