package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import Constants.EnumConstants.LimelightMode;
import Constants.LimelightConstants;
import subsystems.Limelight;
import utility.RobotHardware;

/**
 * Switches the Limelight to the motif AprilTag pipeline (5), waits for a
 * tag (21-23), then switches back to the localization pipeline (2).
 *
 * On successful detection, switches the spindexer to Sorted mode so the
 * upcoming shots use the motif pattern.
 */
public class LimelightScanCommand extends CommandBase {
    private final Limelight limelight;
    private final double timeoutSeconds;
    private ElapsedTime timer;
    private boolean timedOut;

    public static String lastDebug = "no scan yet";

    public static final double DEFAULT_TIMEOUT = 5.0;
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

        Limelight3A ll = RobotHardware.getInstance().limelight;
        if (ll != null) {
            if (!ll.isRunning()) ll.start();
            ll.pipelineSwitch(LimelightConstants.MOTIF_PIPELINE);
        }

        limelight.resetMotifDetection();
        limelight.setMode(LimelightMode.TagTracking);

        lastDebug = "init -> pipeline " + LimelightConstants.MOTIF_PIPELINE;
    }

    @Override
    public void execute() {
        Limelight3A ll = RobotHardware.getInstance().limelight;
        if (ll != null) {
            ll.pipelineSwitch(LimelightConstants.MOTIF_PIPELINE);
        }

        if (timer.seconds() < PIPELINE_SETTLE_SECONDS) return;

        limelight.updateLimelightData();
        limelight.scanForMotifTag();

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
        Limelight3A ll = RobotHardware.getInstance().limelight;
        if (ll != null) {
            ll.pipelineSwitch(LimelightConstants.LOCALIZATION_PIPELINE);
        }
        limelight.setMode(LimelightMode.GoalTracking);

        lastDebug = "end -> pipeline " + LimelightConstants.LOCALIZATION_PIPELINE
                + " (motifDetected=" + limelight.isMotifDetected()
                + ", tagId=" + limelight.getDetectedTagId()
                + ", timedOut=" + timedOut
                + ", elapsed=" + String.format("%.2f", timer.seconds()) + "s)";
    }
}
