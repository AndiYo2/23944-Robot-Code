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
import Constants.SensorConstants;

import java.util.List;

public class RobotHardware {

    // ******************* DRIVE TRAIN ******************* //
    public DcMotorEx frontLeft, backLeft, frontRight, backRight;

    // ******************* PARK ******************* //
    public Servo kickServo, beamServo;


    // ******************* LOCALIZERS ******************* //
    public GoBildaPinpointDriver pinpoint;


    // ******************* INTAKE ******************* //
    public DcMotorEx intakeMotor, intakeBeltMotor;

    // ******************* SHOOTER ******************* //
    public DcMotorEx shooterMotor1;
    public DcMotorEx shooterMotor2;
    public Servo shooterFlipper;

    public AnalogInput shooterFlipperEncoder;
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

    public AnalogInput spindexerFlipperEncoder;
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

    // ******************* SMART DISTANCE-BASED POLLING ******************* //
    // Scan order: intake (back-most) → transfer → ramp (front-most)
    private DualBallDetector[] smartScanOrder;

    // ******************* CACHED PINPOINT POSE ******************* //
    public double cachedPoseX;
    public double cachedPoseY;
    public double cachedHeading;
    public double cachedVelX;
    public double cachedVelY;
    public double cachedHeadingVel;
    public volatile boolean relocalizationPending = false;

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

        // ******************* PARK ******************* //
        kickServo = hardwareMap.get(Servo.class, NamingConstants.Drivetrain.kickServo);
        beamServo = hardwareMap.get(Servo.class, NamingConstants.Drivetrain.beamServo);

        // ******************* PINPOINT ******************* //
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, NamingConstants.Pinpoint.pinpoint);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setOffsets(-0.5, 36.5, DistanceUnit.MM); // Y pod: 0.5mm right, X pod: 36.5mm forward of CoR

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
        intakeSensorPair = new DualBallDetector(intakeSensor1, intakeSensor2,
                new double[]{0.15, 0.55, 0.30}, new double[]{0.32, 0.22, 0.46},
                0.15, 0.15, 400, 250,
                SensorConstants.INTAKE_NEAR_DIST_THRESHOLD_MM,
                SensorConstants.INTAKE_FAR_DIST_THRESHOLD_MM);

        rampSensor1 = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.rampSensor1);
        rampSensor2 = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.rampSensor2);
        rampSensorPair = new DualBallDetector(rampSensor1, rampSensor2,
                new double[]{0.15, 0.55, 0.30}, new double[]{0.32, 0.22, 0.46},
                0.15, 0.15, 400, 250,
                SensorConstants.RAMP_NEAR_DIST_THRESHOLD_MM,
                SensorConstants.RAMP_FAR_DIST_THRESHOLD_MM);

        transferSensor1 = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.transferSensor1);
        transferSensor2 = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.transferSensor2);
        transferSensorPair = new DualBallDetector(transferSensor1, transferSensor2,
                new double[]{0.21, 0.47, 0.32},   // transfer green profile
                new double[]{0.32, 0.31, 0.37},   // transfer purple profile
                0.15, 0.15,                        // green/purple tolerance
                200, 400,                          // near/far alpha thresholds
                SensorConstants.TRANSFER_NEAR_DIST_THRESHOLD_MM,
                SensorConstants.TRANSFER_FAR_DIST_THRESHOLD_MM);

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

        // Smart scan: back-most empty first (intake → transfer → ramp)
        smartScanOrder = new DualBallDetector[]{intakeSensorPair, transferSensorPair, rampSensorPair};

        // ******************* SPINDEXER ******************* //
        spindexerFlipperServo = hardwareMap.get(Servo.class, NamingConstants.Spindexer.spindexerFlipperServo);
        spindexerServo = hardwareMap.get(ServoImplEx.class, NamingConstants.Spindexer.spindexerServo);
        spindexerFlipperEncoder = hardwareMap.get(AnalogInput.class, NamingConstants.Spindexer.spindexerFlipperEncoder);
        spindexerServo.setPwmRange(new PwmControl.PwmRange(500, 2500));  // Full range for Axon at 6V

        // ******************* OUTTAKE ******************* //
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, NamingConstants.Shooter.shooter1);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, NamingConstants.Shooter.shooter2);
        shooterMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        turretServo = hardwareMap.get(Servo.class, NamingConstants.Turret.turret);
        shooterFlipper = hardwareMap.get(Servo.class, NamingConstants.Shooter.shooterFlipperServo);
        shooterFlipperEncoder =  hardwareMap.get(AnalogInput.class, NamingConstants.Shooter.shooterFlipperEncoder);
        shooterHood = hardwareMap.get(Servo.class, NamingConstants.Shooter.shooterHood);


        // ******************* LIMELIGHT ******************* //
         limelight = hardwareMap.get(Limelight3A.class, NamingConstants.Limelight.limelight);
        limelight.setPollRateHz(30);
        limelight.pipelineSwitch(2);
        limelight.start();

        // ******************* VOLTAGE SENSOR ******************* //
        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } else {
            voltageSensor = null; // Will need null check when used
        }

        resetCachedState();
    }

    public void resetCachedState() {
        cachedPoseX = 0;
        cachedPoseY = 0;
        cachedHeading = 0;
        cachedVelX = 0;
        cachedVelY = 0;
        cachedHeadingVel = 0;
        relocalizationPending = false;
        roundRobinIndex = 0;

        // Clear smart polling distance detection state
        if (smartScanOrder != null) {
            for (DualBallDetector detector : smartScanOrder) {
                detector.setDistanceDetected(false);
            }
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

    /** Read ALL color sensors every cycle using bulk-cache-safe reads (3 pairs × 2 sensors). */
    public void pollAllSensors() {
        intakeSensorPair.updateCacheBulkSafe();
        transferSensorPair.updateCacheBulkSafe();
        rampSensorPair.updateCacheBulkSafe();
    }

    /**
     * Smart distance-based sensor polling. Two phases:
     * Phase 1: If any pair is mid-color-burst, tick it (1 bulk RGBA read).
     * Phase 2: Distance scan — find first empty slot (back to front), check distance.
     *          If ball detected, start color burst. If ball departed, clear detection.
     */
    public void smartPollSensors() {
        // Phase 1: Service any active color burst
        for (DualBallDetector detector : smartScanOrder) {
            if (detector.isInColorBurst()) {
                detector.colorBurstTick();
                return;
            }
        }

        // Phase 2: Distance scan — find first pair where ball is not yet confirmed
        for (DualBallDetector detector : smartScanOrder) {
            if (!detector.quickCheck().ballPresent) {
                // Slot appears empty — check distance for incoming ball
                if (detector.checkDistancePresent()) {
                    detector.setDistanceDetected(true);
                    detector.startColorBurst(SensorConstants.COLOR_READ_CYCLES);
                }
                return;
            } else if (detector.isDistanceDetected() && !detector.checkDistancePresent()) {
                // Ball was present but distance now clear — ball has departed
                detector.setDistanceDetected(false);
                return;
            }
        }
    }

    /** Read pinpoint pose/velocities once and cache for the entire loop. */
    public void updateCachedPose() {
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        if (status != GoBildaPinpointDriver.DeviceStatus.READY) {
            return; // Keep last good cached values
        }

        double newX = pinpoint.getPosX(DistanceUnit.INCH);
        double newY = pinpoint.getPosY(DistanceUnit.INCH);

        // After relocalization, allow the first large jump through
        if (relocalizationPending) {
            relocalizationPending = false;
        } else {
            // Reject teleportation: >12 inches in one loop is physically impossible
            double dx = newX - cachedPoseX;
            double dy = newY - cachedPoseY;
            if ((cachedPoseX != 0 || cachedPoseY != 0) && (dx * dx + dy * dy > 144)) {
                return; // Keep last good values
            }
        }

        cachedPoseX = newX;
        cachedPoseY = newY;
        cachedHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        cachedVelX = pinpoint.getVelX(DistanceUnit.INCH);
        cachedVelY = pinpoint.getVelY(DistanceUnit.INCH);
        cachedHeadingVel = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }
}
