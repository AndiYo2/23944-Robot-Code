package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import utility.RobotHardware;
import vision.ArtifactDetector;
import vision.ChosenPath;
import vision.LaneSelector;
import vision.VisionConstants;

/**
 * Scans for balls using the vision system, picks the best lane,
 * and follows the corresponding collection path.
 * Ends when the path completes or 3 balls are detected (spindexer full).
 */
public class VisionCollectCommand extends CommandBase {
    private final ArtifactDetector detector;
    private final Follower follower;
    private final PathChain[] goToPaths;
    private final double maxPower;
    private ChosenPath chosenPath;

    /** Readable from telemetry after the scan runs. */
    public static String lastScanResult = "No scan yet";

    public VisionCollectCommand(ArtifactDetector detector, Follower follower,
                                 PathChain[] goToPaths, double maxPower) {
        this.detector = detector;
        this.follower = follower;
        this.goToPaths = goToPaths;
        this.maxPower = maxPower;
    }

    @Override
    public void initialize() {
        chosenPath = LaneSelector.selectPath(detector, follower.getPose());
        lastScanResult = LaneSelector.lastScanDebug;

        follower.setMaxPower(maxPower);
        follower.followPath(goToPaths[chosenPath.ordinal()], false);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy() || isFull();
    }

    @Override
    public void end(boolean interrupted) {
        follower.breakFollowing();
    }

    public ChosenPath getChosenPath() {
        return chosenPath;
    }

    private boolean isFull() {
        RobotHardware hw = RobotHardware.getInstance();
        return hw.spindexerSensorPair.quickCheck().ballPresent
                && hw.rampSensorPair.quickCheck().ballPresent
                && hw.transferSensorPair.quickCheck().ballPresent;
    }
}
