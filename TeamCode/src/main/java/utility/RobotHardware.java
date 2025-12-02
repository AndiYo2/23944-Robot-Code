package utility;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class RobotHardware {
    // Drivetrain
    public DcMotorEx frontLeft, backLeft, frontRight, backRight;


    // Hardware
    public IMU imu;
    private HardwareMap hardwareMap;
    private static RobotHardware instance = null;
    public boolean enabled = false;


    // Intake
    public DcMotorEx intakeMotor, intakeBeltMotor;

    // Shooter
    public DcMotorEx shooterMotor, shooterBeltMotor;
    public AnalogInput shooterEncoder;
    public CRServo turretServo;

    //Driver Controller
    public GamepadEx driver;

    // Limelight
    public Limelight3A limelight;

    // Color Sensor
    public ColorSensor colorSensorIntake;

    //Spindexer
    public CRServo spindexerMotor;
    public AnalogInput spindexerEncoder;

    public TelemetryManager telemetryManager;

    public Servo spindexerServo;



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

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE); // MAYBE CHANGE
        backLeft.setDirection(DcMotorEx.Direction.REVERSE); // MAYBE CHANGE

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, //
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();


        // ******************* INTAKE ******************* //
        intakeMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Intake.intake);
        intakeBeltMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Intake.intakeBelt);
        intakeBeltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeBeltMotor.setDirection(DcMotor.Direction.REVERSE);

        colorSensorIntake = hardwareMap.get(ColorSensor.class, RobotConstants.ColorSensor.colorSensor);

        // ******************* SPINDEXER ******************* //

        spindexerMotor = hardwareMap.get(CRServo.class, RobotConstants.Spindexer.spindexer);
        spindexerEncoder = hardwareMap.get(AnalogInput.class, RobotConstants.Spindexer.spindexerEncoder);
        spindexerServo = hardwareMap.get(Servo.class, RobotConstants.Spindexer.spindexerFLipperServo);


        // ******************* OUTTAKE ******************* //
        shooterMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Shooter.shooter);
        shooterBeltMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Shooter.outtakeBelt);
        shooterBeltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretServo = hardwareMap.get(CRServo.class, RobotConstants.Shooter.turret);
        shooterEncoder = hardwareMap.get(AnalogInput.class, RobotConstants.Shooter.shooterEncoder);

        // ******************* LIMELIGHT ******************* //
         limelight = hardwareMap.get(Limelight3A.class, RobotConstants.Limelight.limelight);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

    }
}
