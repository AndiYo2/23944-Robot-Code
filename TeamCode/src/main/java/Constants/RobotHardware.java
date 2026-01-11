package Constants;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import utility.DualBallDetector;
import Constants.NamingConstants;
import Constants.OdometryConstants;
import Constants.SpindexerConstants;

public class RobotHardware {

    // ******************* DRIVE TRAIN ******************* //
    public DcMotorEx frontLeft, backLeft, frontRight, backRight;


    // ******************* LOCALIZERS ******************* //
    public IMU imu;
    public GoBildaPinpointDriver pinpoint;


    // ******************* INTAKE ******************* //
    public DcMotorEx intakeMotor, intakeBeltMotor;

    // ******************* SHOOTER ******************* //
    public DcMotorEx shooterMotor1;
    public DcMotorEx shooterMotor2;
    public Servo shooterFlipper;
    public AnalogInput shooterEncoder;
    public CRServo turretServo;
    public AnalogInput turretEncoder;


    // ******************* LIMELIGHT ******************* //
    public Limelight3A limelight;

    // ******************* COLOR SENSORS ******************* //
    // Intake sensors (2 sensors offset at first spindexer slot to avoid ball holes)
    public ColorSensor intakeSensor1;
    public ColorSensor intakeSensor2;
    public DualBallDetector intakeSensorPair;


    // ******************* SPINDEXER ******************* //
    public CRServo spindexerServo;
    public Servo spindexerFlipperServo;
    public AnalogInput spindexerEncoder;

    public PIDFController spindexerPID;

    // ******************* VOLTAGE SENSOR ******************* //
    public VoltageSensor voltageSensor;

    // ******************* GAME CONTROL ******************* //
    public GamepadEx driver;
    public TelemetryManager telemetryManager;
    private HardwareMap hardwareMap;
    private static RobotHardware instance = null;
    public boolean enabled = false;



    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(final HardwareMap hardwareMap, GamepadEx driver) {
        this.driver = driver;
        init(hardwareMap);
    }

    public void init(final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();



        // ******************* DRIVETRAIN ******************* //
        frontLeft = hardwareMap.get(DcMotorEx.class, NamingConstants.Drivetrain.frontLeftMotor);
        backLeft = hardwareMap.get(DcMotorEx.class, NamingConstants.Drivetrain.backLeftMotor);
        backRight = hardwareMap.get(DcMotorEx.class, NamingConstants.Drivetrain.backRightMotor);
        frontRight = hardwareMap.get(DcMotorEx.class, NamingConstants.Drivetrain.frontRightMotor);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        // ******************* IMU ******************* //
        // NOTE: Control Hub IMU initialized but not used - we use Pinpoint's built-in IMU instead
//        imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, //
//                RevHubOrientationOnRobot.UsbFacingDirection.UP));
//        imu.initialize(parameters);
//        imu.resetYaw();

        // ******************* PINPOINT ******************* //
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, NamingConstants.Pinpoint.pinpoint);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setOffsets(0.0, 0.0, DistanceUnit.INCH); // Centered by design in CAD

        // CRITICAL: Reset and calibrate IMU
        // Robot MUST be stationary during this! Calibration takes ~250ms
        // This resets position to (0,0,0) and calibrates the IMU to current orientation
        pinpoint.resetPosAndIMU();

        // Wait for calibration to complete before proceeding
        try {
            Thread.sleep(300); // 250ms calibration + 50ms buffer
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        // Set yaw scalar for fine-tuning (1.0 = no scaling, tune if needed)
        pinpoint.setYawScalar(OdometryConstants.yawScalar);

        // Note: OpModes will set their starting position AFTER this init completes
        // - Auton: Calls follower.setStartingPose() which updates the Pinpoint
        // - TeleOp: Calls pinpoint.setPosition() with either auton ending pose or default start point

        // ******************* INTAKE ******************* //
        intakeMotor = hardwareMap.get(DcMotorEx.class, NamingConstants.Intake.intake);
        intakeBeltMotor = hardwareMap.get(DcMotorEx.class, NamingConstants.Intake.intakeBelt);
        intakeBeltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeBeltMotor.setDirection(DcMotor.Direction.REVERSE);

        // ******************* COLOR SENSORS ******************* //
        // Intake sensors (2 offset sensors at first spindexer slot to avoid ball holes)
        intakeSensor1 = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.intakeSensor1);
        intakeSensor2 = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.intakeSensor2);
        intakeSensorPair = new DualBallDetector(intakeSensor1, intakeSensor2);


        // ******************* SPINDEXER ******************* //
        spindexerFlipperServo = hardwareMap.get(Servo.class, NamingConstants.Spindexer.spindexerFlipperServo);

        spindexerEncoder = hardwareMap.get(AnalogInput.class, NamingConstants.Spindexer.spindexerEncoder);
        spindexerServo = hardwareMap.get(CRServo.class, NamingConstants.Spindexer.spindexerServo);
        spindexerServo.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexerPID = new PIDFController(SpindexerConstants.SPINDEXER_P, SpindexerConstants.SPINDEXER_I, SpindexerConstants.SPINDEXER_D, SpindexerConstants.SPINDEXER_F);

        // ******************* OUTTAKE ******************* //
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, NamingConstants.Shooter.shooter1);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, NamingConstants.Shooter.shooter2);
        shooterMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        turretServo = hardwareMap.get(CRServo.class, NamingConstants.Turret.turret);
        turretEncoder = hardwareMap.get(AnalogInput.class, NamingConstants.Turret.turretEncoder);
        turretServo.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterEncoder = hardwareMap.get(AnalogInput.class, NamingConstants.Shooter.shooterEncoder);
        shooterFlipper = hardwareMap.get(Servo.class, NamingConstants.Shooter.shooterFlipperServo);


        // ******************* LIMELIGHT ******************* //
         limelight = hardwareMap.get(Limelight3A.class, NamingConstants.Limelight.limelight);
        limelight.setPollRateHz(30);
        limelight.pipelineSwitch(5);
        limelight.start();

        // ******************* VOLTAGE SENSOR ******************* //
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }
}
