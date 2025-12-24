package utility;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public AnalogInput turretEncoder;


    // ******************* LIMELIGHT ******************* //
    public Limelight3A limelight;

    // ******************* COLOR SENSORS ******************* //
    // Intake sensors (2 sensors offset at first spindexer slot to avoid ball holes)
    public ColorSensor intakeSensor1;
    public ColorSensor intakeSensor2;

    // Dual sensor reader (combines 2 sensors for reliable detection)
    public DualColorSensor intakeSensor;

    // ******************* SPINDEXER ******************* //
    public CRServo spindexerServo;
    public Servo spindexerFlipperServo;
    public AnalogInput spindexerEncoder;
    public RobotConstants.SpindexerPattern spindexerPattern;

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

        // ******************* COLOR SENSORS ******************* //
        // Intake sensors (2 offset sensors at first spindexer slot to avoid ball holes)
        intakeSensor1 = hardwareMap.get(ColorSensor.class, RobotConstants.ColorSensor.intakeSensor1);
        intakeSensor2 = hardwareMap.get(ColorSensor.class, RobotConstants.ColorSensor.intakeSensor2);
        intakeSensor = new DualColorSensor(intakeSensor1, intakeSensor2);

        // ******************* SPINDEXER ******************* //
        spindexerFlipperServo = hardwareMap.get(Servo.class, RobotConstants.Spindexer.spindexerFLipperServo);
        spindexerPattern = new RobotConstants.SpindexerPattern(BallColor.None, BallColor.None, BallColor.None);
        spindexerEncoder = hardwareMap.get(AnalogInput.class, RobotConstants.Spindexer.spindexerEncoder);
        spindexerServo = hardwareMap.get(CRServo.class, RobotConstants.Spindexer.spindexerServo);
        spindexerServo.setDirection(DcMotorSimple.Direction.REVERSE);

        // ******************* OUTTAKE ******************* //
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, RobotConstants.Shooter.shooter1);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, RobotConstants.Shooter.shooter2);
        shooterMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        turretServo = hardwareMap.get(CRServo.class, RobotConstants.Shooter.turret);
        turretEncoder = hardwareMap.get(AnalogInput.class, RobotConstants.Shooter.turretEncoder);
        shooterEncoder = hardwareMap.get(AnalogInput.class, RobotConstants.Shooter.shooterEncoder);
        shooterFlipper = hardwareMap.get(Servo.class, RobotConstants.Shooter.shooterFlipperServo);


        // ******************* LIMELIGHT ******************* //
         limelight = hardwareMap.get(Limelight3A.class, RobotConstants.Limelight.limelight);
        limelight.setPollRateHz(30);
        limelight.pipelineSwitch(0);
        limelight.start();


    }
}
