package utility;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
public class RobotHardware {
    // Drivetrain
    public DcMotorEx frontLeft, backLeft, frontRight, backRight;
    public IMU imu;

    // Hardware
    private HardwareMap hardwareMap;
    private static RobotHardware instance = null;
    private boolean enabled;

    // Intake
    public DcMotorEx intakeMotor, intakeBeltMotor;

    // Outtake
    public DcMotorEx shooterMotor, shooterBeltMotor;
    public CRServo turretServo;

    //Driver Controller
    public GamepadEx driver;

    // Limelight
    public Limelight3A limelight;

    // Color Sensor
    public ColorSensor colorSensor;

    //Spindexer
    public DcMotorEx spindexerMotor;

    public TelemetryManager telemetryManager;



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
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, //
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        imu.resetYaw();


        // ******************* INTAKE ******************* //
        intakeMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Intake.intake);
        intakeBeltMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Intake.intakeBelt);
        intakeBeltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        colorSensor = hardwareMap.get(ColorSensor.class, RobotConstants.Intake.colorSensor);

        // ******************* SPINDEXER ******************* //
        spindexerMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Spindexer.spindexer);


        // ******************* OUTTAKE ******************* //
        shooterMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Outtake.shooter);
        shooterBeltMotor = hardwareMap.get(DcMotorEx.class, RobotConstants.Outtake.outtakeBelt);
        shooterBeltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretServo = hardwareMap.get(CRServo.class, RobotConstants.Outtake.turret);

        // ******************* LIMELIGHT ******************* //
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.setPollRateHz(100);
//        limelight.pipelineSwitch(0);
//        limelight.start();

    }
}
