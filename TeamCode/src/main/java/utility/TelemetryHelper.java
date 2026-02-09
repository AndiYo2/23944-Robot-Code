package utility;

import Constants.RobotConstants;
import Constants.SpindexerConstants;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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

        // Status
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

        // Position — use cached pose values
        double x = robot.cachedPoseX;
        double y = robot.cachedPoseY;
        double heading = Math.toDegrees(robot.cachedHeading);
        panels.debug("Pose: (" + (int)(x * 10) / 10.0 + ", " + (int)(y * 10) / 10.0 + ") " + (int)(heading * 10) / 10.0 + "°");
        panels.debug("Spind: " + spindexer.getCurrentDegrees() + "°");

        // Targeting
        panels.debug("Turret: " + (int)(turretAngle * 10) / 10.0 + "°");
        panels.debug("Hood: " + (int)(hoodAngle * 10) / 10.0 + "°");
        panels.debug("Dist: " + (int)(distance * 10) / 10.0 + " in");
        panels.debug("Vel: " + (int) currentVel + " / " + (int) targetVel + " tks");

        // Sensors
        DualBallDetector.Result r1 = robot.intakeSensorPair.quickCheck();
        DualBallDetector.Result r2 = robot.transferSensorPair.quickCheck();
        DualBallDetector.Result r3 = robot.rampSensorPair.quickCheck();
        panels.debug("Intake:   " + (r1.ballPresent ? "BALL" : "----") + " " + r1.color + " " + (int)(r1.confidence * 100) + "%");
        panels.debug("Transfer: " + (r2.ballPresent ? "BALL" : "----") + " " + r2.color + " " + (int)(r2.confidence * 100) + "%");
        panels.debug("Ramp:     " + (r3.ballPresent ? "BALL" : "----") + " " + r3.color + " " + (int)(r3.confidence * 100) + "%");

        // Graphs
        panels.addData("Shooter Velocity (actual)", currentVel);
        panels.addData("Shooter Velocity (target)", targetVel);
        panels.addData("Velocity Error", shooter.getVelocityError());
        panels.addData("Hood Angle", hoodAngle);
        panels.addData("Distance to Target", distance);


        panels.addData("Turret Target Angle", turretAngle);
        panels.addData("Turret Degrees to Goal", turret.getLastDegreesToGoal());
        panels.addData("Turret Raw Angle", turret.getRawTargetDegrees());

        if (robot.voltageSensor != null) {
            panels.addData("Battery Voltage", robot.voltageSensor.getVoltage());
        }

        panels.update(telemetry);
    }
}
