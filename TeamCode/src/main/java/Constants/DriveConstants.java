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

    public static double PARK_SERVO_EXTEND = 0.62;
    public static double PARK_SERVO_RETRACT = 0.32;

    public static double BEAM_SERVO_EXTENDED = .8;
    public static double BEAM_SERVO_RETRACTED = 0.33;
}
