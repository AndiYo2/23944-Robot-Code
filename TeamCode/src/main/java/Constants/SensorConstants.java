package Constants;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class SensorConstants {
    public static double INTAKE_NEAR_DIST_THRESHOLD_MM = 25.0;   // ball max 11.2, none min 36
    public static double INTAKE_FAR_DIST_THRESHOLD_MM = 25.0;    // ball max 11.9, none min 35.5

    public static double TRANSFER_NEAR_DIST_THRESHOLD_MM = 28.0; // ball max 20.4, none min 36
    public static double TRANSFER_FAR_DIST_THRESHOLD_MM = 28.0;  // ball max 15.8, none min 41.5

    public static double RAMP_NEAR_DIST_THRESHOLD_MM = 22.0;     // ball max 11.2, none min 31
    public static double RAMP_FAR_DIST_THRESHOLD_MM = 25.0;      // ball max 16.5, none min 52

    // Number of RGBA read cycles after distance trigger
    public static int COLOR_READ_CYCLES = 5;
}
