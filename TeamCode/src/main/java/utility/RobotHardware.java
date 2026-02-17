package utility;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import Constants.NamingConstants;
import Constants.OdometryConstants;

import java.util.List;

public class RobotHardware {

    // ******************* DRIVE TRAIN ******************* //
    public DcMotorEx frontLeft, backLeft, frontRight, backRight;


    // ******************* LOCALIZERS ******************* //
    public GoBildaPinpointDriver pinpoint;


    // ******************* INTAKE ******************* //
    public DcMotorEx intakeMotor, intakeBeltMotor;

    // ******************* SHOOTER ******************* //
    public DcMotorEx shooterMotor1;
    public DcMotorEx shooterMotor2;
    public Servo shooterFlipper;
    public Servo shooterHood;
    public Servo turretServo;


    // ******************* LIMELIGHT ******************* //
    public Limelight3A limelight;

    // ******************* COLOR SENSORS ******************* //
    // Intake sensors (2 sensors offset at first spindexer slot to avoid ball holes)
    public ColorSensor intakeSensor1;
    public ColorSensor intakeSensor2;
    public DualBallDetector intakeSensorPair;

    public ColorSensor rampSensor1;
    public ColorSensor rampSensor2;
    public DualBallDetector rampSensorPair;

    public ColorSensor transferSensor1;
    public ColorSensor transferSensor2;
    public DualBallDetector transferSensorPair;


    // ******************* SPINDEXER ******************* //
    public ServoImplEx spindexerServo;  // Using ServoImplEx for PWM range control
    public Servo spindexerFlipperServo;

    // ******************* VOLTAGE SENSOR ******************* //
    public VoltageSensor voltageSensor;

    // ******************* GAME CONTROL ******************* //
    public GamepadEx driver;
    public TelemetryManager telemetryManager;
    private HardwareMap hardwareMap;

    // Thread-safe singleton pattern
    private static volatile RobotHardware instance = null;
    private static final Object lock = new Object();

    // Volatile to ensure visibility across threads
    public volatile boolean enabled = false;

    // ******************* BULK CACHING ******************* //
    private List<LynxModule> allHubs;

    // ******************* ROUND-ROBIN SENSOR POLLING ******************* //
    private DualBallDetector[] sensorDetectors;
    private boolean[] isNearSensor;
    private int roundRobinIndex = 0;

    // ******************* CACHED PINPOINT POSE ******************* //
    public double cachedPoseX;
    public double cachedPoseY;
    public double cachedHeading;
    public double cachedVelX;
    public double cachedVelY;
    public double cachedHeadingVel;

    /**
     * Returns the singleton instance of RobotHardware.
     * Thread-safe implementation using double-checked locking.
     */
    public static RobotHardware getInstance() {
        if (instance == null) {
            synchronized (lock) {
                if (instance == null) {
                    instance = new RobotHardware();
                }
            }
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

        // ******************* BULK CACHING ******************* //
        // MANUAL mode: we clear the cache once at the top of each loop,
        // so all reads within that loop hit the same bulk-read snapshot.
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // ******************* DRIVETRAIN ******************* //
        frontLeft = hardwareMap.get(DcMotorEx.class, NamingConstants.Drivetrain.frontLeftMotor);
        backLeft = hardwareMap.get(DcMotorEx.class, NamingConstants.Drivetrain.backLeftMotor);
        backRight = hardwareMap.get(DcMotorEx.class, NamingConstants.Drivetrain.backRightMotor);
        frontRight = hardwareMap.get(DcMotorEx.class, NamingConstants.Drivetrain.frontRightMotor);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

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
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeBeltMotor = hardwareMap.get(DcMotorEx.class, NamingConstants.Intake.intakeBelt);
        intakeBeltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeBeltMotor.setDirection(DcMotor.Direction.REVERSE);

        // ******************* COLOR SENSORS ******************* //
        // Intake sensors (2 offset sensors at first spindexer slot to avoid ball holes)
        intakeSensor1 = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.intakeSensor1);
        intakeSensor2 = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.intakeSensor2);
        intakeSensorPair = new DualBallDetector(intakeSensor1, intakeSensor2);

        rampSensor1 = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.rampSensor1);
        rampSensor2 = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.rampSensor2);
        rampSensorPair = new DualBallDetector(rampSensor1, rampSensor2);

        transferSensor1 = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.transferSensor1);
        transferSensor2 = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.transferSensor2);
        transferSensorPair = new DualBallDetector(transferSensor1, transferSensor2,
                new double[]{0.21, 0.47, 0.32},   // transfer green profile
                new double[]{0.32, 0.31, 0.37},   // transfer purple profile
                0.15, 0.15,                        // green/purple tolerance
                200, 400);                         // near/far alpha thresholds

        // Set all detectors to background mode for round-robin polling
        intakeSensorPair.setBackgroundMode(true);
        transferSensorPair.setBackgroundMode(true);
        rampSensorPair.setBackgroundMode(true);

        // Round-robin: 6 sensors, cycling near/far across 3 detector pairs
        sensorDetectors = new DualBallDetector[]{
            intakeSensorPair, intakeSensorPair,
            rampSensorPair, rampSensorPair,
            transferSensorPair, transferSensorPair
        };
        isNearSensor = new boolean[]{true, false, true, false, true, false};
        roundRobinIndex = 0;

        // ******************* SPINDEXER ******************* //
        spindexerFlipperServo = hardwareMap.get(Servo.class, NamingConstants.Spindexer.spindexerFlipperServo);
        spindexerServo = hardwareMap.get(ServoImplEx.class, NamingConstants.Spindexer.spindexerServo);
        spindexerServo.setPwmRange(new PwmControl.PwmRange(500, 2500));  // Full range for Axon at 6V

        // ******************* OUTTAKE ******************* //
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, NamingConstants.Shooter.shooter1);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, NamingConstants.Shooter.shooter2);
        shooterMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        turretServo = hardwareMap.get(Servo.class, NamingConstants.Turret.turret);
        shooterFlipper = hardwareMap.get(Servo.class, NamingConstants.Shooter.shooterFlipperServo);
        shooterHood = hardwareMap.get(Servo.class, NamingConstants.Shooter.shooterHood);


        // ******************* LIMELIGHT ******************* //
         limelight = hardwareMap.get(Limelight3A.class, NamingConstants.Limelight.limelight);
        limelight.setPollRateHz(30);
        limelight.pipelineSwitch(5);
        limelight.start();

        // ******************* VOLTAGE SENSOR ******************* //
        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } else {
            voltageSensor = null; // Will need null check when used
        }
    }

    /** Clear bulk cache on all hubs. Call once at the top of each loop. */
    public void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    /** Read ONE color sensor in round-robin order (6 sensors total). Call once per loop. */
    public void pollNextSensor() {
        if (isNearSensor[roundRobinIndex]) {
            sensorDetectors[roundRobinIndex].updateNearCache();
        } else {
            sensorDetectors[roundRobinIndex].updateFarCache();
        }
        roundRobinIndex = (roundRobinIndex + 1) % 6;
    }

    /** Read pinpoint pose/velocities once and cache for the entire loop. */
    public void updateCachedPose() {
        cachedPoseX = pinpoint.getPosX(DistanceUnit.INCH);
        cachedPoseY = pinpoint.getPosY(DistanceUnit.INCH);
        cachedHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        cachedVelX = pinpoint.getVelX(DistanceUnit.INCH);
        cachedVelY = pinpoint.getVelY(DistanceUnit.INCH);
        cachedHeadingVel = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }
}
