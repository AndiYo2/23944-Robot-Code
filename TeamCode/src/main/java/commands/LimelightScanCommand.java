package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import Constants.EnumConstants.LimelightMode;
import subsystems.Limelight;
import utility.RobotHardware;

/**
 * Command that toggles the limelight to scanning mode and waits for motif detection.
 * Completes when the limelight has detected a motif OR timeout is reached.
 */
public class LimelightScanCommand extends CommandBase {
    private final Limelight limelight;
    private final double timeoutSeconds;
    private ElapsedTime timer;
    private boolean timedOut;

    /** Default timeout for motif scanning (seconds) */
    public static final double DEFAULT_TIMEOUT = 5.0;

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
    }

    /** Returns true if the command completed due to timeout rather than detection */
    public boolean didTimeout() {
        return timedOut;
    }
}
