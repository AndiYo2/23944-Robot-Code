package utility;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.camerastream.PanelsCameraStream;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Scalar;
import vision.VisionConstants;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import Constants.EnumConstants;
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

    // ******************* WEBCAM ******************* //
    public WebcamName autonVisionCamera;
    public VisionPortal visionPortal;
    public ColorBlobLocatorProcessor greenBlobProcessor;
    public ColorBlobLocatorProcessor purpleBlobProcessor;

    // ******************* COLOR SENSORS ******************* //
    // Intake sensors (2 sensors offset at first spindexer slot to avoid ball holes)
    public ColorSensor spindexerSensor1;
    public ColorSensor spindexerSensor2;
    public DualBallDetector spindexerSensorPair;

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

    // ******************* PROGRESSIVE SCAN (AUTO) ******************* //
    // Scan order: spindexer → transfer → ramp (one pair at a time)
    private DualBallDetector[] progressiveScanOrder;
    private EnumConstants.SensorPairState[] progressivePairState;
    private int progressiveScanIndex = 0;

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

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        spindexerSensor1 = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.spindexerSensor1);
        spindexerSensor2 = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.spindexerSensor2);
        spindexerSensorPair = new DualBallDetector(spindexerSensor1, spindexerSensor2,
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
        spindexerSensorPair.setBackgroundMode(true);
        transferSensorPair.setBackgroundMode(true);
        rampSensorPair.setBackgroundMode(true);

        // Round-robin: 6 sensors, cycling near/far across 3 detector pairs
        sensorDetectors = new DualBallDetector[]{
                spindexerSensorPair, spindexerSensorPair,
            rampSensorPair, rampSensorPair,
            transferSensorPair, transferSensorPair
        };
        isNearSensor = new boolean[]{true, false, true, false, true, false};
        roundRobinIndex = 0;

        // Smart scan: back-most empty first (intake → transfer → ramp)
        smartScanOrder = new DualBallDetector[]{spindexerSensorPair, transferSensorPair, rampSensorPair};

        // Progressive scan order for auto: spindexer → transfer → ramp
        progressiveScanOrder = new DualBallDetector[]{spindexerSensorPair, transferSensorPair, rampSensorPair};
        progressivePairState = new EnumConstants.SensorPairState[]{
                EnumConstants.SensorPairState.UNCHECKED,
                EnumConstants.SensorPairState.UNCHECKED,
                EnumConstants.SensorPairState.UNCHECKED
        };
        progressiveScanIndex = 0;

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

        // ******************* WEBCAM (AutonVisionCamera) ******************* //
        autonVisionCamera = hardwareMap.get(WebcamName.class, NamingConstants.Camera.autonVisionCamera);

        // Custom HSV color ranges — tuned at venue, more reliable than SDK defaults
        ColorRange greenRange = new ColorRange(
                ColorSpace.HSV,
                new Scalar(VisionConstants.HSV_GREEN_H_LO, VisionConstants.HSV_GREEN_S_LO, VisionConstants.HSV_GREEN_V_LO),
                new Scalar(VisionConstants.HSV_GREEN_H_HI, VisionConstants.HSV_GREEN_S_HI, VisionConstants.HSV_GREEN_V_HI));

        ColorRange purpleRange = new ColorRange(
                ColorSpace.HSV,
                new Scalar(VisionConstants.HSV_PURPLE_H_LO, VisionConstants.HSV_PURPLE_S_LO, VisionConstants.HSV_PURPLE_V_LO),
                new Scalar(VisionConstants.HSV_PURPLE_H_HI, VisionConstants.HSV_PURPLE_S_HI, VisionConstants.HSV_PURPLE_V_HI));

        greenBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(greenRange)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 0.2, 1, -1))
                .setDrawContours(false)
                .setBlurSize(9)
                .build();

        purpleBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(purpleRange)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 0.2, 1, -1))
                .setDrawContours(false)
                .setBlurSize(9)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(autonVisionCamera)
                .setCameraResolution(new Size(
                        VisionConstants.VISION_PORTAL_WIDTH,
                        VisionConstants.VISION_PORTAL_HEIGHT))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(greenBlobProcessor)
                .addProcessor(purpleBlobProcessor)
                .build();

        PanelsCameraStream.INSTANCE.startStream(visionPortal, 75);

        // ******************* VOLTAGE SENSOR ******************* //
        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } else {
            voltageSensor = null; // Will need null check when used
        }

        resetCachedState();
    }

    /** Stop the auton vision portal and camera stream. Call from TeleOp init. */
    public void stopVisionPortal() {
        if (visionPortal != null) {
            PanelsCameraStream.INSTANCE.stopStream();
            visionPortal.close();
            visionPortal = null;
            greenBlobProcessor = null;
            purpleBlobProcessor = null;
        }
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

        // Reset progressive scan state
        resetProgressiveScan();
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
        spindexerSensorPair.updateCacheBulkSafe();
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

    // ==================== Progressive Scan (Auto) ====================

    /**
     * Progressive auto sensor polling. Scans one pair at a time:
     * spindexer → transfer → ramp. Each pair goes through:
     * UNCHECKED → (distance check) → COLOR_SCANNING → CONFIRMED.
     * Stops scanning once all 3 are CONFIRMED.
     */
    public void progressivePollAuto() {
        if (isProgressiveScanComplete()) return;

        // Service any active color burst first
        for (int i = 0; i < progressiveScanOrder.length; i++) {
            if (progressivePairState[i] == EnumConstants.SensorPairState.COLOR_SCANNING) {
                DualBallDetector detector = progressiveScanOrder[i];
                if (detector.isInColorBurst()) {
                    detector.colorBurstTick();
                    return;
                }
                // Burst finished — check if color was identified
                if (detector.quickCheck().color != EnumConstants.BallColor.None) {
                    progressivePairState[i] = EnumConstants.SensorPairState.CONFIRMED;
                    // Advance to next unconfirmed pair
                    advanceProgressiveIndex();
                } else {
                    // No color — go back to UNCHECKED to retry
                    progressivePairState[i] = EnumConstants.SensorPairState.UNCHECKED;
                }
                return;
            }
        }

        // Check distance on the current pair
        if (progressiveScanIndex < progressiveScanOrder.length) {
            DualBallDetector detector = progressiveScanOrder[progressiveScanIndex];
            if (progressivePairState[progressiveScanIndex] == EnumConstants.SensorPairState.UNCHECKED) {
                if (detector.checkDistancePresent()) {
                    // Ball detected — start color burst
                    detector.setDistanceDetected(true);
                    detector.startColorBurst(SensorConstants.COLOR_READ_CYCLES);
                    progressivePairState[progressiveScanIndex] = EnumConstants.SensorPairState.COLOR_SCANNING;
                }
            }
        }
    }

    /** Reset progressive scan to initial state. Call after shooting. */
    public void resetProgressiveScan() {
        if (progressivePairState != null) {
            for (int i = 0; i < progressivePairState.length; i++) {
                progressivePairState[i] = EnumConstants.SensorPairState.UNCHECKED;
            }
        }
        progressiveScanIndex = 0;
        if (progressiveScanOrder != null) {
            for (DualBallDetector detector : progressiveScanOrder) {
                detector.setDistanceDetected(false);
            }
        }
    }

    /** Returns true when all 3 sensor pairs have confirmed ball color. */
    public boolean isProgressiveScanComplete() {
        if (progressivePairState == null) return false;
        for (EnumConstants.SensorPairState state : progressivePairState) {
            if (state != EnumConstants.SensorPairState.CONFIRMED) return false;
        }
        return true;
    }

    private void advanceProgressiveIndex() {
        for (int i = 0; i < progressivePairState.length; i++) {
            if (progressivePairState[i] != EnumConstants.SensorPairState.CONFIRMED) {
                progressiveScanIndex = i;
                return;
            }
        }
        progressiveScanIndex = progressivePairState.length; // All confirmed
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
