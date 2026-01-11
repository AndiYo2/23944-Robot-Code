package Constants;

/**
 * Hardware device names for the robot configuration.
 *
 * PORT COMMENTS:
 *   - E = Expansion Hub
 *   - C = Control Hub
 *   - Numbers indicate port indices
 */
public class NamingConstants {

    public static class Drivetrain {
        public static String frontLeftMotor = "frontLeftMotor";   // E0
        public static String backLeftMotor = "backLeftMotor";     // E1
        public static String frontRightMotor = "frontRightMotor"; // E2
        public static String backRightMotor = "backRightMotor";   // E3
    }

    public static class Intake {
        public static String intake = "intakeMotor";       // C3
        public static String intakeBelt = "intakeBeltMotor"; // C2
    }

    public static class Spindexer {
        public static String spindexerServo = "spindexerServo";           // E4
        public static String spindexerEncoder = "spindexerServoEncoder";  // E Analog 0/1?
        public static String spindexerFlipperServo = "spindexerFlipperServo"; // E5
    }

    public static class Shooter {
        public static String shooter1 = "shooterMotor1";           // Left flywheel (from back), C0
        public static String shooter2 = "shooterMotor2";           // Right flywheel (from back), C1
        public static String shooterFlipperServo = "shooterFlipperServo"; // Ball flipper servo, C0
        public static String shooterEncoder = "shooterEncoder";
    }

    public static class Turret {
        public static String turret = "turretServo";               // Continuous rotation servo, E3
        public static String turretEncoder = "turretServoEncoder"; // E Analog 0/1?
    }

    public static class ColorSensor {
        public static String intakeSensor1 = "intakeSensor1"; // First intake sensor
        public static String intakeSensor2 = "intakeSensor2"; // Second intake sensor (offset from first)
    }

    public static class Limelight {
        public static String limelight = "limelight";
    }

    public static class Pinpoint {
        public static String pinpoint = "pinpoint"; // E I2C 3
    }
}
