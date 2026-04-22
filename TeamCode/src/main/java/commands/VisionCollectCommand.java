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
import vision.FieldBall;
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
        if (!usable || !follower.isBusy()) return true;
        if (isFull()) return true;
        if (passedLastBall()) return true;
        return false;
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

    /**
     * Spindexer is full: all three sensors (spindexer + ramp + transfer)
     * see a ball simultaneously. Strict 3-of-3 — we want every ball we can
     * get. Missing a 3rd ball is lost points.
     */
    private boolean isFull() {
        RobotHardware hw = RobotHardware.getInstance();
        return hw.spindexerSensorPair.quickCheck().ballPresent
                && hw.rampSensorPair.quickCheck().ballPresent
                && hw.transferSensorPair.quickCheck().ballPresent;
    }

    /**
     * Geometric safety-net: only triggers if we've driven the full corridor
     * past the last targeted ball by CORRIDOR_FINISH_BUFFER_IN. This is the
     * "sensors are late but we've literally run out of corridor" bail-out,
     * not an early-stop mechanism. Raising CORRIDOR_FINISH_BUFFER_IN makes
     * the robot more willing to wait for slow sensor reads at the cost of
     * potential overshoot.
     */
    private boolean passedLastBall() {
        if (sweep == null || sweep.captured == null || sweep.captured.isEmpty()) return false;
        FieldBall last = sweep.captured.get(sweep.captured.size() - 1);
        double ax = Math.cos(sweep.headingRad);
        double ay = Math.sin(sweep.headingRad);
        Pose pose = follower.getPose();
        double dx = pose.getX() - last.fieldX;
        double dy = pose.getY() - last.fieldY;
        double alongPastBall = dx * ax + dy * ay;
        return alongPastBall > VisionConstants.CORRIDOR_FINISH_BUFFER_IN;
    }
}
