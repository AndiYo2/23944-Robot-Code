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
 * Vision collection with single-scan + fallback.
 *
 * State machine:
 *   1. initialize: scan at current pose.
 *        balls found → drive corridor   (state = DRIVE_CORRIDOR)
 *        no balls    → fallback path    (state = FALLBACK)
 *
 *   2. execute (after drive finishes): state = DONE
 *
 * Finishes when state is DONE, or when the spindexer is full (3-of-3
 * sensors), or when the follower finishes naturally.
 *
 * NOTE: secondaryHeadingRad is retained in the constructor signature for
 * call-site compatibility but is no longer used.
 */
public class VisionCollectWithFallbacksCommand extends CommandBase {

    private final ArtifactDetector detector;
    private final Follower follower;
    private final double maxPower;
    private final double secondaryHeadingRad;
    private final PathChain fallbackPath;

    private enum State {
        SCAN,
        DRIVE_CORRIDOR,
        FALLBACK,
        DONE
    }

    private State state = State.SCAN;
    private CorridorPlanner.Sweep lastSweep = CorridorPlanner.Sweep.EMPTY;

    /** Readable from telemetry. */
    public static String lastResult = "No scan yet";

    public VisionCollectWithFallbacksCommand(
            ArtifactDetector detector,
            Follower follower,
            double maxPower,
            double secondaryHeadingRad,
            PathChain fallbackPath) {
        this.detector = detector;
        this.follower = follower;
        this.maxPower = maxPower;
        this.secondaryHeadingRad = secondaryHeadingRad;
        this.fallbackPath = fallbackPath;
    }

    @Override
    public void initialize() {
        state = State.SCAN;
        CorridorPlanner.Sweep sweep = CorridorSelector.selectCorridor(detector, follower.getPose());
        lastSweep = sweep;

        if (hasUsableSweep(sweep)) {
            driveCorridor(sweep);
            state = State.DRIVE_CORRIDOR;
            lastResult = "scan OK: " + corridorSummary(sweep);
        } else {
            follower.setMaxPower(maxPower);
            follower.followPath(fallbackPath, false);
            state = State.FALLBACK;
            lastResult = "scan EMPTY → fallback path";
        }
    }

    @Override
    public void execute() {
        if (follower.isBusy()) return;

        switch (state) {
            case DRIVE_CORRIDOR:
            case FALLBACK:
                state = State.DONE;
                break;

            case SCAN:
            case DONE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        if (state == State.DONE) return true;

        if ((state == State.DRIVE_CORRIDOR || state == State.FALLBACK) && isFull()) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (follower.isBusy()) {
            follower.breakFollowing();
        }
    }

    public CorridorPlanner.Sweep getLastSweep() {
        return lastSweep;
    }

    // ----- Helpers -----

    private boolean hasUsableSweep(CorridorPlanner.Sweep sweep) {
        return sweep != null
                && !sweep.isEmpty()
                && sweep.score >= VisionConstants.CORRIDOR_MIN_SCORE;
    }

    private void driveCorridor(CorridorPlanner.Sweep sweep) {
        Pose start = follower.getPose();
        PathChain pc = follower.pathBuilder()
                .addPath(new BezierLine(start, sweep.endPose))
                .setLinearHeadingInterpolation(start.getHeading(), sweep.endPose.getHeading())
                .build();
        follower.setMaxPower(maxPower);
        follower.followPath(pc, false);
    }

    private boolean isFull() {
        RobotHardware hw = RobotHardware.getInstance();
        return hw.spindexerSensorPair.quickCheck().ballPresent
                && hw.rampSensorPair.quickCheck().ballPresent
                && hw.transferSensorPair.quickCheck().ballPresent;
    }

    private static String corridorSummary(CorridorPlanner.Sweep s) {
        int cap = s.captured == null ? 0 : s.captured.size();
        return String.format("hdg=%.0f° cap=%d score=%.2f",
                Math.toDegrees(s.headingRad), cap, s.score);
    }
}
