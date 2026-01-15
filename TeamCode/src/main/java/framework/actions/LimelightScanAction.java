package framework.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import framework.Action;
import subsystems.Limelight;
import Constants.EnumConstants;
import utility.RobotHardware;

/**
 * Action that toggles the limelight to scanning mode and waits for motif detection.
 * Completes when the limelight has detected a motif OR timeout is reached.
 */
public class LimelightScanAction implements Action {
    private final Limelight limelight;
    private final double timeoutSeconds;
    private ElapsedTime timer;
    private boolean timedOut;

    /** Default timeout for motif scanning (seconds) */
    public static final double DEFAULT_TIMEOUT = 5.0;

    public LimelightScanAction(Limelight limelight) {
        this(limelight, DEFAULT_TIMEOUT);
    }

    public LimelightScanAction(Limelight limelight, double timeoutSeconds) {
        this.limelight = limelight;
        this.timeoutSeconds = timeoutSeconds;
        this.timer = new ElapsedTime();
        this.timedOut = false;
    }

    @Override
    public void start() {
        timer.reset();
        timedOut = false;
        if (!RobotHardware.getInstance().limelight.isRunning()) {
            RobotHardware.getInstance().limelight.start();
        }
        limelight.setMode(EnumConstants.LimelightMode.TagTracking);
    }

    @Override
    public void update() {
        // Check for timeout
        if (timer.seconds() >= timeoutSeconds && !limelight.isMotifDetected()) {
            timedOut = true;
        }
    }

    @Override
    public boolean isComplete() {
        return limelight.isMotifDetected() || timedOut;
    }

    @Override
    public void end() {
        // Nothing to clean up
    }

    /** Returns true if the action completed due to timeout rather than detection */
    public boolean didTimeout() {
        return timedOut;
    }

    @Override
    public String getName() {
        return timedOut ? "LimelightScan(TIMEOUT)" : "LimelightScan";
    }
}
