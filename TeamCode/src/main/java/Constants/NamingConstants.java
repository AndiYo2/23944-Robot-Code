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
        public static String kickServo = "kickServo";
        public static String beamServo = "beamServo";
    }

    public static class Intake {
        public static String intake = "intakeMotor";       // C3
        public static String intakeBelt = "intakeBeltMotor"; // C2
    }

    public static class Spindexer {
        public static String spindexerServo = "spindexerServo";           // E4

        public static String spindexerFlipperEncoder = "spindexerFlipperEncoder";
        public static String spindexerFlipperServo = "spindexerFlipperServo"; // E5
    }

    public static class Shooter {
        public static String shooter1 = "shooterMotor1";           // Left flywheel (from back), C0
        public static String shooter2 = "shooterMotor2";           // Right flywheel (from back), C1
        public static String shooterFlipperServo = "shooterFlipperServo"; // Ball flipper servo, C0

        public static String shooterFlipperEncoder = "shooterFlipperEncoder";
        public static String shooterHood = "shooterHoodServo";     // Hood angle servo, E?
    }

    public static class Turret {
        public static String turret = "turretServo";  // Position servo, E3
    }

    public static class ColorSensor {
        public static String intakeSensor1 = "spindexerSensor1"; // E1
        public static String intakeSensor2 = "spindexerSensor2";  // E2
        public static String rampSensor1 = "rampSensor1"; // CH 0
        public static String rampSensor2 = "rampSensor2";  // CH 1 ( V3 )
        public static String transferSensor1 = "transferSensor1"; // CH 2
        public static String transferSensor2 = "transferSensor2";  // CH 3
    }

    public static class Limelight {
        public static String limelight = "limelight";
    }

    public static class Pinpoint {
        public static String pinpoint = "pinpoint"; // E I2C 3
    }
}
