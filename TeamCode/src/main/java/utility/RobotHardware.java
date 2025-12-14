package utility;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import utility.RobotConstants.Enums.BallColor;

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


    // ******************* LIMELIGHT ******************* //
    public Limelight3A limelight;

    // ******************* COLOR SENSOR ******************* //
    public ColorSensor colorSensor;

    // ******************* SPINDEXER ******************* //
    public CRServo spindexerServo;
    public Servo spindexerFlipperServo;
    public AnalogInput spindexerEncoder;
    public RobotConstants.SpindxerPattern spindexerPattern;

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
        frontLeft = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.frontLeftMotor);
        backLeft = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.backLeftMotor);
        backRight = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.backRightMotor);
        frontRight = hardwareMap.get(DcMotorEx.class, RobotConstants.Drivetrain.frontRightMotor);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        // ******************* IMU ******************* //
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, //
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        // ******************* PINPOINT ******************* //
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setOffsets(0.0, 0.0, DistanceUnit.INCH);

        // ******************* INTAKE ******************* //
        intakeMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Intake.intake);
        intakeBeltMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Intake.intakeBelt);
        intakeBeltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeBeltMotor.setDirection(DcMotor.Direction.REVERSE);

        // ******************* COLOR SENSOR ******************* //
        colorSensor = hardwareMap.get(ColorSensor.class, RobotConstants.ColorSensor.colorSensor);

        // ******************* SPINDEXER ******************* //
        spindexerFlipperServo = hardwareMap.get(Servo.class, RobotConstants.Spindexer.spindexerFLipperServo);
        spindexerPattern = new RobotConstants.SpindxerPattern(BallColor.None, BallColor.None, BallColor.None);
        spindexerEncoder = hardwareMap.get(AnalogInput.class, RobotConstants.Spindexer.spindexerEncoder);
        spindexerServo = hardwareMap.get(CRServo.class, RobotConstants.Spindexer.spindexerServo);
        spindexerServo.setDirection(DcMotorSimple.Direction.FORWARD);

        // ******************* OUTTAKE ******************* //
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, RobotConstants.Shooter.shooter1);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, RobotConstants.Shooter.shooter2);
        shooterMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        turretServo = hardwareMap.get(CRServo.class, RobotConstants.Shooter.turret);
        shooterEncoder = hardwareMap.get(AnalogInput.class, RobotConstants.Shooter.shooterEncoder);
        shooterFlipper = hardwareMap.get(Servo.class, RobotConstants.Shooter.shooterFlipperServo);


        // ******************* LIMELIGHT ******************* //
         limelight = hardwareMap.get(Limelight3A.class, RobotConstants.Limelight.limelight);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

    }
}
