package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import subsystems.Limelight;
import utility.RobotHardware;
import utility.SpindexerAndMotifStatus;

/**
 * Scans the classifier ramp via Limelight pipeline 6 (Python SnapScript).
 * Reads getPythonOutput() to count non-zero entries (balls in ramp).
 * Updates RampTracker with the scan result. If scan fails (timeout),
 * RampTracker keeps its existing manual counter value.
 */
public class RampScanCommand extends CommandBase {
    private final Limelight limelight;
    private final double timeoutSeconds;
    private ElapsedTime timer;
    private boolean scanComplete;

    public static final double DEFAULT_TIMEOUT = 1.0;

    public static String lastScanDebug = "no scan yet";
    public static String lastRampReadDebug = "no read yet";

    public RampScanCommand(Limelight limelight) {
        this(limelight, DEFAULT_TIMEOUT);
    }

    public RampScanCommand(Limelight limelight, double timeoutSeconds) {
        this.limelight = limelight;
        this.timeoutSeconds = timeoutSeconds;
        this.timer = new ElapsedTime();
        this.scanComplete = false;
        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        timer.reset();
        scanComplete = false;
        if (!RobotHardware.getInstance().limelight.isRunning()) {
            RobotHardware.getInstance().limelight.start();
        }
    }

    @Override
    public void execute() {
        int count = limelight.readRampBallCount();
        lastRampReadDebug = limelight.lastRampReadDebug;
        if (count >= 0) {
            int oldCount = SpindexerAndMotifStatus.RampTracker.getBallsInRamp();
            SpindexerAndMotifStatus.RampTracker.setBallsInRamp(count);
            scanComplete = true;
            lastScanDebug = String.format("Scan OK: %d balls (was %d)", count, oldCount);
        }
    }

    @Override
    public boolean isFinished() {
        return scanComplete || timer.seconds() >= timeoutSeconds;
    }

    @Override
    public void end(boolean interrupted) {
        if (!scanComplete) {
            lastScanDebug = String.format("Scan TIMEOUT (%.1fs), keeping counter at %d",
                    timeoutSeconds, SpindexerAndMotifStatus.RampTracker.getBallsInRamp());
        }
    }
}
