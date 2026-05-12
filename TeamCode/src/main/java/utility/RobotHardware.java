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
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
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
import java.util.concurrent.TimeUnit;
import Constants.EnumConstants;
import Constants.NamingConstants;
import Constants.OdometryConstants;
import Constants.SensorConstants;

import java.util.List;

public class RobotHardware {

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
    public ServoImplEx spindexerServo;

    public AnalogInput spindexerFlipperEncoder;
    public Servo spindexerFlipperServo;

    // ******************* VOLTAGE SENSOR ******************* //
    public VoltageSensor voltageSensor;

    // ******************* GAME CONTROL ******************* //
    public GamepadEx driver;
    public TelemetryManager telemetryManager;
    private HardwareMap hardwareMap;

    private static volatile RobotHardware instance = null;
    private static final Object lock = new Object();
    public volatile boolean enabled = false;

    // ******************* BULK CACHING ******************* //
    private List<LynxModule> allHubs;

    // ******************* SMART DISTANCE-BASED POLLING ******************* //
    private DualBallDetector[] smartScanOrder;

    // ******************* PROGRESSIVE SCAN (AUTO) ******************* //
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
        init(hardwareMap, driver, true);
    }

    public void init(final HardwareMap hardwareMap, GamepadEx driver, boolean initVisionPortal) {
        this.driver = driver;
        init(hardwareMap, initVisionPortal);
    }

    public void init(final HardwareMap hardwareMap) {
        init(hardwareMap, true);
    }

    public void init(final HardwareMap hardwareMap, boolean initVisionPortal) {
        this.hardwareMap = hardwareMap;
        this.telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        // ******************* BULK CACHING ******************* //
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
        pinpoint.resetPosAndIMU();

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        pinpoint.setYawScalar(OdometryConstants.yawScalar);


        // ******************* INTAKE ******************* //
        intakeMotor = hardwareMap.get(DcMotorEx.class, NamingConstants.Intake.intake);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeBeltMotor = hardwareMap.get(DcMotorEx.class, NamingConstants.Intake.intakeBelt);
        intakeBeltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeBeltMotor.setDirection(DcMotor.Direction.REVERSE);

        // ******************* COLOR SENSORS ******************* //
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

        spindexerSensorPair.setBackgroundMode(true);
        transferSensorPair.setBackgroundMode(true);
        rampSensorPair.setBackgroundMode(true);

        smartScanOrder = new DualBallDetector[]{spindexerSensorPair, transferSensorPair, rampSensorPair};

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
        if (initVisionPortal) {
        autonVisionCamera = hardwareMap.get(WebcamName.class, NamingConstants.Camera.autonVisionCamera);


        ColorRange greenRange  = ColorRange.ARTIFACT_GREEN;
        ColorRange purpleRange = ColorRange.ARTIFACT_PURPLE;

        ImageRegion ballRoi = ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1);

        greenBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(greenRange)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ballRoi)
                .setDrawContours(true)
                .setBlurSize(9)
                .setErodeSize(15)
                .setDilateSize(15)
                .build();

        purpleBlobProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(purpleRange)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ballRoi)
                .setDrawContours(true)
                .setBlurSize(9)
                .setErodeSize(15)
                .setDilateSize(15)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(autonVisionCamera)
                .setCameraResolution(new Size(
                        VisionConstants.VISION_PORTAL_WIDTH,
                        VisionConstants.VISION_PORTAL_HEIGHT))
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(greenBlobProcessor)
                .addProcessor(purpleBlobProcessor)
                .build();

        lockCameraControls();

        if (VisionConstants.PANELS_STREAM_FPS > 0) {
            try {
                PanelsCameraStream.INSTANCE.startStream(visionPortal,
                        VisionConstants.PANELS_STREAM_FPS);
            } catch (Exception ignored) {
            }
        }
        }

        // ******************* VOLTAGE SENSOR ******************* //
        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } else {
            voltageSensor = null;
        }

        resetCachedState();
    }

    private void lockCameraControls() {
        if (visionPortal == null) return;
        long waitStart = System.currentTimeMillis();
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING
               && System.currentTimeMillis() - waitStart < 3000) {
            try { Thread.sleep(20); }
            catch (InterruptedException e) { Thread.currentThread().interrupt(); return; }
        }
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) return;

        try {
            ExposureControl exp = visionPortal.getCameraControl(ExposureControl.class);
            if (exp != null) {
                if (exp.getMode() != ExposureControl.Mode.Manual) {
                    exp.setMode(ExposureControl.Mode.Manual);
                    Thread.sleep(50);
                }
                exp.setExposure(VisionConstants.EXPOSURE_MS, TimeUnit.MILLISECONDS);
                Thread.sleep(20);
            }

            GainControl gain = visionPortal.getCameraControl(GainControl.class);
            if (gain != null) {
                gain.setGain(VisionConstants.GAIN);
                Thread.sleep(20);
            }

            WhiteBalanceControl wb = visionPortal.getCameraControl(WhiteBalanceControl.class);
            if (wb != null) {
                wb.setMode(WhiteBalanceControl.Mode.MANUAL);
                Thread.sleep(50);
                wb.setWhiteBalanceTemperature(VisionConstants.WB_KELVIN);
            }
        } catch (Exception e) {
        }
    }

    public void applyCameraControls() {
        lockCameraControls();
    }

    public void resetCachedState() {
        cachedPoseX = 0;
        cachedPoseY = 0;
        cachedHeading = 0;
        cachedVelX = 0;
        cachedVelY = 0;
        cachedHeadingVel = 0;
        relocalizationPending = false;

        if (smartScanOrder != null) {
            for (DualBallDetector detector : smartScanOrder) {
                detector.setDistanceDetected(false);
            }
        }

        resetProgressiveScan();
    }

    public void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public void smartPollSensors() {
        for (DualBallDetector detector : smartScanOrder) {
            if (detector.isInColorBurst()) {
                detector.colorBurstTick();
                return;
            }
        }

        for (DualBallDetector detector : smartScanOrder) {
            if (!detector.quickCheck().ballPresent) {
                if (detector.checkDistancePresent()) {
                    detector.setDistanceDetected(true);
                    detector.startColorBurst(SensorConstants.COLOR_READ_CYCLES);
                }
                return;
            } else if (detector.isDistanceDetected() && !detector.checkDistancePresent()) {
                detector.setDistanceDetected(false);
                return;
            }
        }
    }

    // ==================== Progressive Scan (Auto) ====================
    public void progressivePollAuto() {
        if (isProgressiveScanComplete()) return;

        for (int i = 0; i < progressiveScanOrder.length; i++) {
            if (progressivePairState[i] == EnumConstants.SensorPairState.COLOR_SCANNING) {
                DualBallDetector detector = progressiveScanOrder[i];
                if (detector.isInColorBurst()) {
                    detector.colorBurstTick();
                    return;
                }
                if (detector.quickCheck().color != EnumConstants.BallColor.None) {
                    progressivePairState[i] = EnumConstants.SensorPairState.CONFIRMED;
                    advanceProgressiveIndex();
                } else {
                    progressivePairState[i] = EnumConstants.SensorPairState.UNCHECKED;
                }
                return;
            }
        }

        if (progressiveScanIndex < progressiveScanOrder.length) {
            DualBallDetector detector = progressiveScanOrder[progressiveScanIndex];
            if (progressivePairState[progressiveScanIndex] == EnumConstants.SensorPairState.UNCHECKED) {
                if (detector.checkDistancePresent()) {
                    detector.setDistanceDetected(true);
                    detector.startColorBurst(SensorConstants.COLOR_READ_CYCLES);
                    progressivePairState[progressiveScanIndex] = EnumConstants.SensorPairState.COLOR_SCANNING;
                }
            }
        }
    }

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
        progressiveScanIndex = progressivePairState.length;
    }

    public void updateCachedPose() {
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        if (status != GoBildaPinpointDriver.DeviceStatus.READY) {
            return;
        }

        double newX = pinpoint.getPosX(DistanceUnit.INCH);
        double newY = pinpoint.getPosY(DistanceUnit.INCH);

        if (relocalizationPending) {
            relocalizationPending = false;
        } else {
            double dx = newX - cachedPoseX;
            double dy = newY - cachedPoseY;
            if ((cachedPoseX != 0 || cachedPoseY != 0) && (dx * dx + dy * dy > 144)) {
                return;
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
