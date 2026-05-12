package Constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class SpindexerConstants {

    // --- Degree-Based Position Constants ---
    // All positions in SERVO degrees (0-300° effective range)
    // Conversion to servo position: degrees / 355.0
    //
    // 6 positions for 3 slots (2 positions per slot due to 2:1 gear ratio):
    //   Slot 0: 0°, 180°
    //   Slot 1: 60°, 240°
    //   Slot 2: 120°, 300°
    public static final int DEGREE_INCREMENT = 60;
    public static final int MAX_SERVO_DEGREES = 300;

    public static final int CW_WRAP_TO_DEG = 180;
    public static final int CCW_WRAP_TO_DEG = 120;

    public static final int EMPTY_RESET_DEGREES = 180;

    public static final double SERVO_DEGREES_PER_UNIT = 355.0;

    public static double SPINDEXER_OFFSET = 10;

    public static double TELEOP_FIRST_CATALOG_INTAKE_TIME = 0.6;
    public static double TELEOP_REVERSE_CATALOG_INTAKE_TIME = 0.5;

    public static double AUTO_CATALOG_STOP_TIME = 0.15;
    public static double AUTO_FIRST_CATALOG_INTAKE_TIME = 0.25;
    public static double AUTO_SECOND_CATALOG_INTAKE_TIME = 0.4;
    public static double AUTO_REVERSE_CATALOG_INTAKE_TIME = 0.35;

    public static double ROTATION_SETTLE_TIME = 0.08;

    public static EnumConstants.ShootingMode currentMode = EnumConstants.ShootingMode.Fast;
}
