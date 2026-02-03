package utility;

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
        if (!RobotConstants.Robot.ENABLE_TELEMETRY) return;

        // ==================== STATUS (debug text) ====================
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
        panels.debug(String.format("Pose: (%.1f, %.1f) %.1f°", x, y, heading));
        panels.debug("Spind: " + spindexer.getCurrentDegrees() + "°");

        // ==================== TARGETING (debug text) ====================
        panels.debug(String.format("Turret: %.1f°", turret.getTargetTurretAngle()));
        panels.debug(String.format("Hood: %.1f°", shooter.getTargetHoodAngle()));
        panels.debug(String.format("Dist: %.1f in", shooter.getDistanceToTarget()));
        panels.debug(String.format("Vel: %.0f / %.0f tks",
                shooter.getCurrentVelocity(), shooter.getTargetVelocity()));

        // ==================== SENSORS (debug text) ====================
        DualBallDetector.Result r1 = robot.intakeSensorPair.detectBall();
        DualBallDetector.Result r2 = robot.transferSensorPair.detectBall();
        DualBallDetector.Result r3 = robot.rampSensorPair.detectBall();
        panels.debug(String.format("Intake:   %s %s %.0f%%",
                r1.ballPresent ? "BALL" : "----", r1.color, r1.confidence * 100));
        panels.debug(String.format("Transfer: %s %s %.0f%%",
                r2.ballPresent ? "BALL" : "----", r2.color, r2.confidence * 100));
        panels.debug(String.format("Ramp:     %s %s %.0f%%",
                r3.ballPresent ? "BALL" : "----", r3.color, r3.confidence * 100));

        // ==================== GRAPHS (Capture time-series) ====================
        // Shooter
        panels.addData("Shooter Velocity (actual)", shooter.getCurrentVelocity());
        panels.addData("Shooter Velocity (target)", shooter.getTargetVelocity());
        panels.addData("Velocity Error", shooter.getVelocityError());
        panels.addData("Hood Angle", shooter.getTargetHoodAngle());
        panels.addData("Distance to Target", shooter.getDistanceToTarget());

        // Controller outputs
        double[] outputs = shooter.getControllerOutputs();
        panels.addData("FF Output", outputs[0]);
        panels.addData("PID Output", outputs[1]);
        panels.addData("Total Power", outputs[2]);

        // Turret
        panels.addData("Turret Target Angle", turret.getTargetTurretAngle());
        panels.addData("Turret Degrees to Goal", turret.getDegreesToGoal());

        // Loop time
        panels.addData("Loop Time (ms)", loopMs);

        // Battery voltage
        if (robot.voltageSensor != null) {
            panels.addData("Battery Voltage", robot.voltageSensor.getVoltage());
        }

        // Driver inputs (TeleOp only -- gamepad is null in Auto)
        if (gamepad != null) {
            panels.addData("Stick Left Y", -gamepad.left_stick_y);
            panels.addData("Stick Left X", gamepad.left_stick_x);
            panels.addData("Stick Right X", gamepad.right_stick_x);
            panels.addData("Trigger Left", gamepad.left_trigger);
            panels.addData("Trigger Right", gamepad.right_trigger);
        }

        // Push to Panels + DS
        panels.update(telemetry);
    }
}
