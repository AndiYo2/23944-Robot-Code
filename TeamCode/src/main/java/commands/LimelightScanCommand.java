package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import Constants.EnumConstants.LimelightMode;
import subsystems.Limelight;
import utility.RobotHardware;

/**
 * Command that toggles the limelight to scanning mode and waits for motif detection.
 * Completes when the limelight has detected a motif OR timeout is reached.
 *
 * This command DOES auto-switch pipelines: motif pipeline on start, localization
 * on end. Contrast with RampScanCommand, which is pipeline-agnostic — callers
 * sequence ramp-pipeline switches manually via SetLimelightPipelineCommand.
 */
public class LimelightScanCommand extends CommandBase {
    private final Limelight limelight;
    private final double timeoutSeconds;
    private ElapsedTime timer;
    private boolean timedOut;

    /** Default timeout for motif scanning (seconds) */
    public static final double DEFAULT_TIMEOUT = 5.0;
    /** Time to wait for pipeline to settle after switching (seconds) */
    private static final double PIPELINE_SETTLE_SECONDS = 0.15;

    public LimelightScanCommand(Limelight limelight) {
        this(limelight, DEFAULT_TIMEOUT);
    }

    public LimelightScanCommand(Limelight limelight, double timeoutSeconds) {
        this.limelight = limelight;
        this.timeoutSeconds = timeoutSeconds;
        this.timer = new ElapsedTime();
        this.timedOut = false;
        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        timer.reset();
        timedOut = false;
        if (!RobotHardware.getInstance().limelight.isRunning()) {
            RobotHardware.getInstance().limelight.start();
        }
        limelight.setMode(LimelightMode.TagTracking);
    }

    @Override
    public void execute() {
        // Wait for pipeline to settle after switching to motif pipeline
        if (timer.seconds() < PIPELINE_SETTLE_SECONDS) return;

        if (timer.seconds() >= timeoutSeconds && !limelight.isMotifDetected()) {
            timedOut = true;
        }
    }

    @Override
    public boolean isFinished() {
        return limelight.isMotifDetected() || timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        // Switch back to localization pipeline (handles timeout case where no tag was found)
        limelight.setMode(LimelightMode.GoalTracking);
    }
}
