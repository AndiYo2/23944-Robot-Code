package utility;

import Constants.RobotConstants;
import Constants.SpindexerConstants;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import subsystems.*;

/**
 * Centralized telemetry helper that sends data to Bylazar Panels.
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

    public TelemetryHelper() {
        this.robot = RobotHardware.getInstance();
        this.panels = robot.telemetryManager;
    }

    public void setSubsystems(Shooter shooter, Turret turret, Spindexer spindexer,
                              Odometry odometry, Limelight limelight) {
        this.shooter = shooter;
        this.turret = turret;
        this.spindexer = spindexer;
        this.odometry = odometry;
        this.limelight = limelight;
    }

    /**
     * Pushes all telemetry data to Panels.
     * Complete no-op when ENABLE_TELEMETRY is false.
     */
    public void update(Telemetry telemetry) {
        if (!RobotConstants.Robot.ENABLE_TELEMETRY) return;

        // ==================== STATUS ====================
        panels.debug("Pattern: " + SpindexerAndMotifStatus.SpindexerPattern.getSpindexerPatternString());
        panels.debug("Mode: " + SpindexerConstants.currentMode);
        panels.debug("Zone: " + odometry.getFieldState());

        if (limelight.isMotifDetected()) {
            panels.debug(String.format("Motif: [%s, %s, %s]",
                    SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(0),
                    SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(1),
                    SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(2)));
        } else {
            panels.debug("Motif: Not Detected");
        }

        panels.debug("LL Mode: " + limelight.getCurrentMode());

        // ==================== POSITION ====================
        double x = robot.pinpoint.getPosX(DistanceUnit.INCH);
        double y = robot.pinpoint.getPosY(DistanceUnit.INCH);
        double heading = Math.toDegrees(robot.pinpoint.getHeading(AngleUnit.RADIANS));
        panels.debug(String.format("Pose: (%.1f, %.1f) %.1f°", x, y, heading));
        panels.debug("Spind: " + spindexer.getCurrentDegrees() + "°");

        // ==================== TARGETING ====================
        panels.debug(String.format("Turret: %.1f°", turret.getTargetTurretAngle()));
        panels.debug(String.format("Hood: %.1f°", shooter.getTargetHoodAngle()));
        panels.debug(String.format("Dist: %.1f in", shooter.getDistanceToTarget()));
        panels.debug(String.format("Vel: %.0f / %.0f tks",
                shooter.getCurrentVelocity(), shooter.getTargetVelocity()));

        // ==================== SENSORS ====================
        DualBallDetector.Result r1 = robot.intakeSensorPair.detectBall();
        DualBallDetector.Result r2 = robot.transferSensorPair.detectBall();
        DualBallDetector.Result r3 = robot.rampSensorPair.detectBall();
        panels.debug(String.format("Intake:   %s %s %.0f%%",
                r1.ballPresent ? "BALL" : "----", r1.color, r1.confidence * 100));
        panels.debug(String.format("Transfer: %s %s %.0f%%",
                r2.ballPresent ? "BALL" : "----", r2.color, r2.confidence * 100));
        panels.debug(String.format("Ramp:     %s %s %.0f%%",
                r3.ballPresent ? "BALL" : "----", r3.color, r3.confidence * 100));

        // Push to Panels
        panels.update(telemetry);
    }
}
