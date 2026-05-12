package utility;

import Constants.RobotConstants;
import Constants.ShooterConstants;
import Constants.SpindexerConstants;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import subsystems.*;


public class TelemetryHelper {
    private final RobotHardware robot;
    private final TelemetryManager panels;

    private Shooter shooter;
    private Turret turret;
    private Spindexer spindexer;
    private Limelight limelight;
    private MecanumDrive mecanumDrive;
    private Intake intake;

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
                              Limelight limelight,
                              MecanumDrive mecanumDrive, Intake intake) {
        this.shooter = shooter;
        this.turret = turret;
        this.spindexer = spindexer;
        this.limelight = limelight;
        this.mecanumDrive = mecanumDrive;
        this.intake = intake;
    }

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


    public void update(Telemetry telemetry, double loopMs) {
        computeLoopStats();

        panels.addData("Loop Time (ms)", loopMs);
        panels.addData("Avg Loop (60s)", rollingAvgMs);
        panels.addData("Max Spike (60s)", rollingMaxMs);
        panels.debug(String.format("Loop: %.1f ms | Avg: %.1f ms | Spike: %.1f ms", loopMs, rollingAvgMs, rollingMaxMs));

        if (!RobotConstants.Robot.ENABLE_TELEMETRY) {
            panels.update(telemetry);
            return;
        }

        double currentVel = shooter.getCurrentVelocity();
        double targetVel = shooter.getTargetVelocity();
        double hoodAngle = shooter.getTargetHoodAngle();
        double distance = shooter.getDistanceToTarget();
        double turretAngle = turret.getTargetTurretAngle();

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


        double x = robot.cachedPoseX;
        double y = robot.cachedPoseY;
        double heading = Math.toDegrees(robot.cachedHeading);
        panels.debug("Pose: (" + (int)(x * 10) / 10.0 + ", " + (int)(y * 10) / 10.0 + ") " + (int)(heading * 10) / 10.0 + "°");

        panels.addData("Shooter Velocity (actual)", currentVel);
        panels.addData("Shooter Velocity (target)", targetVel);
        panels.addData("Velocity Error", shooter.getVelocityError());
        panels.addData("Flywheel Mode", shooter.getFlywheelControlMode().name());
        panels.addData("Hood Angle", hoodAngle);
        panels.addData("Hood Shot Offset", shooter.getShotHoodCompensation());
        panels.addData("Distance to Target", distance);
        panels.addData("Turret Target Angle", turretAngle);
        panels.addData("Velocity Offset (Dpad)", Shooter.runtimeVelocityOffset);
        panels.debug("Manual Override: " + (ShooterConstants.MANUAL_OVERRIDE ? "ON" : "OFF")
                + " | Dpad Vel Offset: " + Shooter.runtimeVelocityOffset);

        panels.update(telemetry);
    }
}
