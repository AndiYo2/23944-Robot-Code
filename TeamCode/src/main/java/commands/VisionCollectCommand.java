package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import utility.RobotHardware;
import vision.ArtifactDetector;
import vision.CorridorPlanner;
import vision.CorridorSelector;
import vision.VisionConstants;

/**
 * Scans for balls, runs the corridor planner, and drives a runtime-built
 * straight-line path through the chosen corridor.
 *
 * Ends when the path completes, the spindexer is full (3 balls), or the
 * planner returned no usable corridor (in which case the command is a no-op
 * and the next command in the sequence runs immediately).
 */
public class VisionCollectCommand extends CommandBase {
    private final ArtifactDetector detector;
    private final Follower follower;
    private final double maxPower;

    private CorridorPlanner.Sweep sweep;
    private boolean usable;

    /** Readable from telemetry after the scan runs. */
    public static String lastScanResult = "No scan yet";

    public VisionCollectCommand(ArtifactDetector detector, Follower follower, double maxPower) {
        this.detector = detector;
        this.follower = follower;
        this.maxPower = maxPower;
    }

    @Override
    public void initialize() {
        this.sweep = CorridorSelector.selectCorridor(detector, follower.getPose());
        lastScanResult = CorridorSelector.lastScanDebug;

        if (sweep.isEmpty() || sweep.score < VisionConstants.CORRIDOR_MIN_SCORE) {
            usable = false;
            return;
        }

        Pose start = follower.getPose();
        PathChain pc = follower.pathBuilder()
                .addPath(new BezierLine(start, sweep.endPose))
                .setLinearHeadingInterpolation(start.getHeading(), sweep.endPose.getHeading())
                .build();

        follower.setMaxPower(maxPower);
        follower.followPath(pc, false);
        usable = true;
    }

    @Override
    public void execute() {
        // Nothing per-tick — follower handles path following.
    }

    @Override
    public boolean isFinished() {
        return !usable || !follower.isBusy() || isFull();
    }

    @Override
    public void end(boolean interrupted) {
        if (usable) {
            follower.breakFollowing();
        }
    }

    public CorridorPlanner.Sweep getSweep() {
        return sweep;
    }

    private boolean isFull() {
        RobotHardware hw = RobotHardware.getInstance();
        return hw.spindexerSensorPair.quickCheck().ballPresent
                && hw.rampSensorPair.quickCheck().ballPresent
                && hw.transferSensorPair.quickCheck().ballPresent;
    }
}
