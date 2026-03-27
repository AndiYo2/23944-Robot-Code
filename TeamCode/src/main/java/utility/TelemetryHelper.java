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

    // Rolling 60-second loop time tracker
    private static final int LOOP_BUFFER_SIZE = 6000; // ~60s at 100 Hz max
    private static final long WINDOW_NS = 60_000_000_000L; // 60 seconds in nanos
    private final double[] loopBuffer = new double[LOOP_BUFFER_SIZE];
    private final long[] loopTimestamps = new long[LOOP_BUFFER_SIZE];
    private int loopHead = 0;  // next write index
    private int loopCount = 0; // total entries in buffer
    private double rollingAvgMs = 0;
    private double rollingMaxMs = 0;

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
     * Records a loop time sample. Call this every loop iteration (not just at telemetry rate)
     * so all samples are captured for accurate avg/spike tracking.
     */
    public void recordLoop(double loopMs) {
        long now = System.nanoTime();
        loopBuffer[loopHead] = loopMs;
        loopTimestamps[loopHead] = now;
        loopHead = (loopHead + 1) % LOOP_BUFFER_SIZE;
        if (loopCount < LOOP_BUFFER_SIZE) loopCount++;
    }

    private void computeLoopStats() {
        long now = System.nanoTime();
        long cutoff = now - WINDOW_NS;
        double sum = 0;
        double max = 0;
        int validCount = 0;

        for (int i = 0; i < loopCount; i++) {
            // Walk the buffer from oldest to newest
            int idx = (loopHead - loopCount + i + LOOP_BUFFER_SIZE) % LOOP_BUFFER_SIZE;
            if (loopTimestamps[idx] < cutoff) continue;
            double val = loopBuffer[idx];
            sum += val;
            if (val > max) max = val;
            validCount++;
        }

        rollingAvgMs = validCount > 0 ? sum / validCount : 0;
        rollingMaxMs = max;
    }

    /**
     * Pushes all telemetry data to Panels (debug text + graph time-series).
     * Complete no-op when ENABLE_TELEMETRY is false.
     *
     * @param telemetry FTC SDK telemetry for Driver Station display
     * @param loopMs    current loop time in milliseconds (for graph tracking)
     */
    public void update(Telemetry telemetry, double loopMs) {
        computeLoopStats();

        // Always report loop timing regardless of ENABLE_TELEMETRY
        panels.addData("Loop Time (ms)", loopMs);
        panels.addData("Avg Loop (60s)", rollingAvgMs);
        panels.addData("Max Spike (60s)", rollingMaxMs);
        panels.debug(String.format("Loop: %.1f ms | Avg: %.1f ms | Spike: %.1f ms", loopMs, rollingAvgMs, rollingMaxMs));

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
        panels.debug("Spindexer Pattern: " + SpindexerAndMotifStatus.SpindexerPattern.getSpindexerPatternString());
        panels.debug("Sorting Mode: " + SpindexerConstants.currentMode);

        if (limelight.isMotifDetected()) {
            panels.debug("Motif: ["
                    + SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(0) + ", "
                    + SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(1) + ", "
                    + SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(2) + "]");
        } else {
            panels.debug("Motif: Not Detected");
        }


        // Position — use cached pose values
        double x = robot.cachedPoseX;
        double y = robot.cachedPoseY;
        double heading = Math.toDegrees(robot.cachedHeading);
        panels.debug("Pose: (" + (int)(x * 10) / 10.0 + ", " + (int)(y * 10) / 10.0 + ") " + (int)(heading * 10) / 10.0 + "°");

        panels.addData("Shooter Velocity (actual)", currentVel);
        panels.addData("Shooter Velocity (target)", targetVel);
        panels.addData("Velocity Error", shooter.getVelocityError());
        panels.addData("Hood Angle", hoodAngle);
        panels.addData("Distance to Target", distance);
        panels.addData("Turret Target Angle", turretAngle);

        panels.update(telemetry);
    }
}
