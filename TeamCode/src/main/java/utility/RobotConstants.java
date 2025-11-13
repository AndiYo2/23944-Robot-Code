package utility;

public class RobotConstants {
    public static class Drivetrain {
        public static String frontLeftMotor = "frontLeftMotor";
        public static String backLeftMotor = "backLeftMotor";
        public static String frontRightMotor= "frontRightMotor";
        public static String backRightMotor = "backRightMotor";
    }

    public static class Intake {
        public static String intake = "intakeMotor";
        public static String intakeBelt = "intakeBeltMotor";
        public static String colorSensor = "colorSensor";

    }

    public static class Spindexer {
        public static String spindexer = "spindexerMotor";

        //Possibly add robot ball patterns here:
    }

    public static class Outtake {
        public static String shooter = "shooterMotor";
        public static String turret = "turretServo";
        public static String outtakeBelt= "outtakeBeltMotor";
    }
}
