package utility;

import Constants.EnumConstants.BallColor;
import Constants.RobotConstants;
import Constants.SpindexerConstants;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import subsystems.*;

/**
 * Centralized telemetry helper that sends data to Bylazar Panels.
 * Uses debug() for text status and graph() for numeric time-series data.
 * Gated behind RobotConstants.Robot.ENABLE_TELEMETRY — when disabled,
 * update() is a complete no-op (no string formatting, no sensor reads).
 */
public class TelemetryHelper {
    private final RobotHardware robot;
    private final TelemetryManager panels;

    private Shooter shooter;
    private Turret turret;
    private Spindexer spindexer;
    private Odometry odometry;
    private Limelight limelight;
    private MecanumDrive mecanumDrive;
    private Intake intake;
    private Gamepad gamepad;

    // Throttle expensive I2C sensor reads (color sensors are not covered by bulk caching)
    private static final int SENSOR_READ_INTERVAL = 5;
    private int loopCount = 0;
    private DualBallDetector.Result cachedIntakeResult = new DualBallDetector.Result(false, BallColor.None, 0.0);
    private DualBallDetector.Result cachedTransferResult = new DualBallDetector.Result(false, BallColor.None, 0.0);
    private DualBallDetector.Result cachedRampResult = new DualBallDetector.Result(false, BallColor.None, 0.0);

    public TelemetryHelper() {
        this.robot = RobotHardware.getInstance();
        this.panels = robot.telemetryManager;
    }

    public void setSubsystems(Shooter shooter, Turret turret, Spindexer spindexer,
                              Odometry odometry, Limelight limelight,
                              MecanumDrive mecanumDrive, Intake intake) {
        this.shooter = shooter;
        this.turret = turret;
        this.spindexer = spindexer;
        this.odometry = odometry;
        this.limelight = limelight;
        this.mecanumDrive = mecanumDrive;
        this.intake = intake;
    }

    public void setGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    /**
     * Pushes all telemetry data to Panels (debug text + graph time-series).
     * Complete no-op when ENABLE_TELEMETRY is false.
     *
     * @param telemetry FTC SDK telemetry for Driver Station display
     * @param loopMs    current loop time in milliseconds (for graph tracking)
     */
    public void update(Telemetry telemetry, double loopMs) {
        // Always report loop timing regardless of ENABLE_TELEMETRY
        panels.addData("Loop Time (ms)", loopMs);

        if (!RobotConstants.Robot.ENABLE_TELEMETRY) {
            panels.update(telemetry);
            return;
        }

        // Cache values used in both debug text and graph sections
        double currentVel = shooter.getCurrentVelocity();
        double targetVel = shooter.getTargetVelocity();
        double hoodAngle = shooter.getTargetHoodAngle();
        double distance = shooter.getDistanceToTarget();
        double turretAngle = turret.getTargetTurretAngle();

        // ==================== STATUS (debug text) ====================
        panels.debug("Pattern: " + SpindexerAndMotifStatus.SpindexerPattern.getSpindexerPatternString());
        panels.debug("Mode: " + SpindexerConstants.currentMode);
        panels.debug("Zone: " + odometry.getFieldState());

        if (limelight.isMotifDetected()) {
            panels.debug("Motif: ["
                    + SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(0) + ", "
                    + SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(1) + ", "
                    + SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(2) + "]");
        } else {
            panels.debug("Motif: Not Detected");
        }

        panels.debug("LL Mode: " + limelight.getCurrentMode());

        if (mecanumDrive != null) {
            panels.debug("Drive: " + mecanumDrive.getCurrentState());
        }
        if (intake != null) {
            panels.debug("Intake: " + intake.getCurrentState());
        }
        panels.debug("Shooter Ready: " + shooter.isAtTargetVelocity());

        // ==================== POSITION (debug text) ====================
        double x = robot.pinpoint.getPosX(DistanceUnit.INCH);
        double y = robot.pinpoint.getPosY(DistanceUnit.INCH);
        double heading = Math.toDegrees(robot.pinpoint.getHeading(AngleUnit.RADIANS));
        panels.debug("Pose: (" + (int)(x * 10) / 10.0 + ", " + (int)(y * 10) / 10.0 + ") " + (int)(heading * 10) / 10.0 + "°");
        panels.debug("Spind: " + spindexer.getCurrentDegrees() + "°");

        // ==================== TARGETING (debug text) ====================
        panels.debug("Turret: " + (int)(turretAngle * 10) / 10.0 + "°");
        panels.debug("Hood: " + (int)(hoodAngle * 10) / 10.0 + "°");
        panels.debug("Dist: " + (int)(distance * 10) / 10.0 + " in");
        panels.debug("Vel: " + (int) currentVel + " / " + (int) targetVel + " tks");

        // ==================== SENSORS (debug text) ====================
        // Color sensors use I2C (not covered by bulk caching) — throttle reads
        loopCount++;
        if (loopCount % SENSOR_READ_INTERVAL == 0) {
            cachedIntakeResult = robot.intakeSensorPair.quickCheck();
            cachedTransferResult = robot.transferSensorPair.quickCheck();
            cachedRampResult = robot.rampSensorPair.quickCheck();
        }
        panels.debug("Intake:   " + (cachedIntakeResult.ballPresent ? "BALL" : "----") + " " + cachedIntakeResult.color + " " + (int)(cachedIntakeResult.confidence * 100) + "%");
        panels.debug("Transfer: " + (cachedTransferResult.ballPresent ? "BALL" : "----") + " " + cachedTransferResult.color + " " + (int)(cachedTransferResult.confidence * 100) + "%");
        panels.debug("Ramp:     " + (cachedRampResult.ballPresent ? "BALL" : "----") + " " + cachedRampResult.color + " " + (int)(cachedRampResult.confidence * 100) + "%");

        // ==================== GRAPHS (Capture time-series) ====================
        // Shooter (reuse cached values from above)
        panels.addData("Shooter Velocity (actual)", currentVel);
        panels.addData("Shooter Velocity (target)", targetVel);
        panels.addData("Velocity Error", shooter.getVelocityError());
        panels.addData("Hood Angle", hoodAngle);
        panels.addData("Distance to Target", distance);

        // Turret (reuse cached turretAngle)
        panels.addData("Turret Target Angle", turretAngle);
        panels.addData("Turret Degrees to Goal", turret.getDegreesToGoal());
        panels.addData("Turret Raw Angle", turret.getRawTargetDegrees());

        // Battery voltage
        if (robot.voltageSensor != null) {
            panels.addData("Battery Voltage", robot.voltageSensor.getVoltage());
        }

        // Push to Panels + DS
        panels.update(telemetry);
    }
}
