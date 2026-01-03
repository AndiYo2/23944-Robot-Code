package utility;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class RobotConstants {
    public static class Drivetrain {
        public static String frontLeftMotor = "frontLeftMotor"; //E0
        public static String backLeftMotor = "backLeftMotor"; // E1
        public static String frontRightMotor= "frontRightMotor"; // E2
        public static String backRightMotor = "backRightMotor"; // E3
    }

    public static class Robot {
        // Robot dimensions (inches)
        public static final double ROBOT_SIZE = 17; // Robot width/length (square)
        public static final double HALF_SIZE = ROBOT_SIZE / 2.0; // Distance from center to edge
    }

    public static class Intake {
        public static String intake = "intakeMotor"; //C 3
        public static String intakeBelt = "intakeBeltMotor"; // C 2

    }

    public static class Spindexer {
        public static SpindexerAndMotifStatus.SpindexerPattern spindexerPattern  = new SpindexerAndMotifStatus.SpindexerPattern(Enums.BallColor.None, Enums.BallColor.None, Enums.BallColor.None);
        public static String spindexerServo = "spindexerServo"; // E4
        public static String spindexerEncoder = "spindexerServoEncoder"; // E Analog 0-1

        public static String spindexerFLipperServo = "spindexerFlipperServo"; // E5
        //Constant Positions
        public final static double FLIPPER_POSITION_EXTENDED = 0.85;
        public final static double FLIPPER_POSITION_RETRACT = 0.51;

        public static final double FLICK_TIME = 0.15;

        public final static PIDCoefficients SPINDEXER_PID = new PIDCoefficients(0.006, 0, 0.00015);

        // Acceptable position error (degrees) - stops rotation when within this range
        public static final double ANGLE_RANGE = 10;

        // Spindexer slot positions in degrees (3 equally-spaced slots: 120° apart)
        public static final int[] SPINDEXER_POSITIONS = {62, 182, 302};

    }


    public static class Shooter {
        //hardware
        public static String shooter1 = "shooterMotor1"; // Left(from back) shooter, C0
        public static String shooter2 = "shooterMotor2"; // Right(from back) shooter, C1
        public static String turret = "turretServo"; //E3
        public static String turretEncoder = "turretServoEncoder"; // Encoder port TBD
        public static String shooterFlipperServo = "shooterFlipperServo"; // C 0
        public static String shooterEncoder = "shooterEncoder"; // N/A

        // 2600, 2100, is our powers

        //Constant Positions
        public final static double FLIPPER_POSITION_EXTENDED = 0.35;
        public final static double FLIPPER_POSITION_RETRACT = 0.15;
        public static final double FLICK_TIME = 0.15;

        // Shooter power settings

        // Turret offset from robot center (in inches, robot-relative)
        public static final double TURRET_OFFSET_X = 4;
        public static final double TURRET_OFFSET_Y = 1;

        // Turret positioning
        public static final double CENTER = 0.0;
        public static final int ENCODER_OFFSET = -30; // Encoder offset: position that reads -60° raw is 0° actual
        public static final double ANGLE_RANGE = 0.5; // Acceptable error in degrees
        public static final double GEAR_RATIO = 6.0; // 6:1 servo to turret (servo rotates 6° for 1° turret rotation)

        // Turret tracking offset (in turret degrees) - compensates for systematic tracking error
        // Negative value shifts aim left, positive shifts aim right
        public static final double TURRET_TRACKING_OFFSET = 0; // Adjust if tracking is still off

        // Turret PID coefficients (tuned values)
        public static final PIDCoefficients TURRET_PID = new PIDCoefficients(0.013, 0, 0.00020);

        // Shooter PIDF coefficients (tuned values from ShooterPIDFTuningTeleOp)
        public static final double SHOOTER_P = 5.0;
        public static final double SHOOTER_I = 0.0;
        public static final double SHOOTER_D = 0.0;
        public static final double SHOOTER_F = 6.4;
    }

    public static class ColorSensor {
        // Intake sensors (2 sensors offset to avoid ball holes at first spindexer slot)
        public static String intakeSensor1 = "intakeSensor1"; // First intake sensor
        public static String intakeSensor2 = "intakeSensor2"; // Second intake sensor (offset from first)

        // Color detection thresholds
        public static final double PURPLE_THRESHOLD = 700;
        public static final double GREEN_THRESHOLD = 1500;
        public static final double ALPHA_THRESHOLD = 500;

        // Detection logic: Green ball detected if (green > red) AND (green > GREEN_THRESHOLD)
    }

    public static class Controls {
        // Gamepad trigger activation threshold
        public static final double TRIGGER_THRESHOLD = 0.3;
    }

    public static class Limelight{

        public static String limelight = "limelight";
        public static boolean isLimelightDisabled = false;
        public static SpindexerAndMotifStatus.MotifPattern motifPattern = new SpindexerAndMotifStatus.MotifPattern(Enums.BallColor.Green, Enums.BallColor.Purple, Enums.BallColor.Purple);
    }

    public static class Pinpoint{
        public static String pinpoint = "pinpoint"; //E I2C 1
        public static Pose2D standardStartPoint = new Pose2D(DistanceUnit.INCH,56.5, 8.5, AngleUnit.DEGREES, 90);
    }
    public static class UpdatableConstants{
        public static double shooterVelocity;
        public static Pose endingAutonPose;

        public static Enums.AllianceColor allianceColor;
    }



    public static class Enums{
        public enum FieldState{
            IdleZone,
            ShootingZone,
            ParkZone,
            PenaltyZone
        }
        public enum AllianceColor {
            Red,
            Blue
        }
        public enum FlickState {
            Idle,
            Start,
            Extended,
            Retracted
        }
        public enum RotationState {
            IDLE,      // Ready for commands
            ROTATING   // Busy rotating
        }
        public enum ShooterCases{
            Idle,
            Start,
            SpindexerFlicking,
            ShooterFlicking,
            SpindexerRotating,
            BallShot

        }
        public enum BallColor{
            Purple,
            Green,
            None
        }
        public enum CatalogingCases{
            Idle,
            Scanning,
            WaitingForRotation,
            RotateToEndLocation
        }
        public enum IntakeState {
            Idle,           // Motors stopped
            Intaking,       // Both motors forward (runs continuously)
            Reversing,      // Both motors backward (eject)
            StagingOnly,    // Only staging belt
            IntakeOnly      // Only intake motor
        }
        public enum DriveState {
            Idle,               // Not moving
            FieldRelative,      // Field-centric (default)
            RobotRelative,      // Robot-centric
            SlowMode,           // Precision mode
            AutoDriving,        // Autonomous navigation
            Locked              // Defense mode (X-pattern)
        }
    }
}
