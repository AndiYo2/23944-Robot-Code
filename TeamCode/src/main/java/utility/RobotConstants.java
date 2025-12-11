package utility;

public class RobotConstants {
    public static class Drivetrain {
        public static String frontLeftMotor = "frontLeftMotor";
        public static String backLeftMotor = "backLeftMotor";
        public static String frontRightMotor= "frontRightMotor";
        public static String backRightMotor = "backRightMotor";
        public static double DRIVE_HEADING = 0;
    }

    public static class Intake {
        public static String intake = "intakeMotor";
        public static String intakeBelt = "intakeBeltMotor";

    }

    public static class Spindexer {
        public static String spindexer = "spindexerServo";
        public static String spindexerEncoder = "spindexerServoEncoder";
        public static String spindexerMotor = "spindexerServoEncoder2";

        public static String spindexerFLipperServo = "spindexerFlipperServo";
        //Constant Positions
        public final static double FLIPPER_POSITION_EXTENDED = 0.65;
        public final static double FLIPPER_POSITION_RETRACT = 0.25;

    }


    public static class Shooter {
        //hardware
        public static String shooter = "shooterMotor";
        public static String turret = "turretServo";
        public static String shooterFlipperServo = "shooterFlipperServo";
        public static String shooterEncoder = "shooterEncoder";

        //Constant Positions
        public final static double FLIPPER_POSITION_EXTENDED = 0.6;
        public final static double FLIPPER_POSITION_RETRACT = 0.325;




    }

    public static class ColorSensor {
        public static String colorSensor = "colorSensor";
    }

    public static class Limelight{

        public static String limelight = "limelight";
        public static boolean isLimelightDisabled = false;
        public static MotiffPattern motiffPatern = new MotiffPattern(Enums.BallColor.None, Enums.BallColor.None, Enums.BallColor.None);
        public static final double LIMELIGHT_HEIGHT = 0.41; // Height of limelight from ground in meters
        public static final double LIMELIGHT_ANGLE = 10.0; // Angle of limelight from horizontal in degrees
        public static final double APRILTAG_HEIGHT = 0.75; // Height of AprilTag center from ground (could be 1.0m - verify this!)
        public static final double TARGET_OFFSET = .1; // How much higher than AprilTag we want to aim (meters)
        public static final double TARGET_HEIGHT = APRILTAG_HEIGHT + TARGET_OFFSET; // Actual target height (1.0m or 1.25m)
        public static final double LAUNCH_HEIGHT = 0.38; // Height of ball launch point from ground in meters
        public static final double LAUNCH_ANGLE = 40.0; // Launch angle in degrees (adjust based on your hood)
        public static final double GRAVITY = 9.81; // m/s^2
        public static final double DEADBAND = 8.0;  //within 8 degrees
    }

    public static class Pinpoint{
        public static String pinpoint = "pinpoint";
    }

    public static class MotiffPattern{
        public static Enums.BallColor[] ballPattern;
        public MotiffPattern(Enums.BallColor zero, Enums.BallColor one, Enums.BallColor two){
            ballPattern = new Enums.BallColor[]{zero, one, two};
        }
        public Enums.BallColor getBallColorInSlotX(int x){
            return ballPattern[x];
        }
        public void setBallPattern(Enums.BallColor[] pattern){
            ballPattern = pattern;
        }
        public void setBallPattern(Enums.BallColor zero, Enums.BallColor one, Enums.BallColor two){
            ballPattern = new Enums.BallColor[]{zero, one, two};
        }
    }

    public static class SpindxerPattern{
        // ZERO - Intake
        // ONE - Outtake
        // TWO - Top Slot

        public static Enums.BallColor[] spindexerPattern;
        public SpindxerPattern(Enums.BallColor zero, Enums.BallColor one, Enums.BallColor two){
            spindexerPattern = new Enums.BallColor[]{zero, one, two};
        }
        public Enums.BallColor getBallInSlotX(int x){
            return spindexerPattern[x];
        }
        public void setBallPatternNone(int x){
            spindexerPattern[x] = Enums.BallColor.None;
        }
        public void emptyBallPattern(){
            spindexerPattern = new Enums.BallColor[]{Enums.BallColor.None, Enums.BallColor.None, Enums.BallColor.None};
        }
        public void setBallInSlotX(int x, Enums.BallColor ballType){
            spindexerPattern[x] = ballType;
        }
        public void setBallPattern(Enums.BallColor[] pattern){
            spindexerPattern = pattern;
        }
        public void setBallPattern(Enums.BallColor zero, Enums.BallColor one, Enums.BallColor two){
            spindexerPattern = new Enums.BallColor[]{zero, one, two};
        }




    }

    public static class Enums{
        public enum FlickState {
            Idle,
            Start,
            Extended,
            Retracted
        }
        public enum ShooterCases{
            Idle,
            Start,
            SpindexerFlicking,
            ShooterFlicking,
            SpindexerRotating

        }

        public enum BallColor{
            Purple,
            Green,
            None
        }
    }



}
