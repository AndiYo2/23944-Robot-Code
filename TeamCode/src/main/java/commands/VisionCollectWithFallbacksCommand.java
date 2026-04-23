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
 * Vision collection with a two-stage scan and a fallback drive.
 *
 * State machine (no Thread.sleep, no blocking — each transition happens
 * from execute() when the follower is idle):
 *
 *   1. initialize:       run first scan at current pose.
 *        balls found → drive corridor          (state = DRIVE_CORRIDOR)
 *        no balls    → rotate in place         (state = ROTATE)
 *
 *   2. execute (after ROTATE finishes):
 *        run second scan at current pose.
 *          balls found → drive corridor       (state = DRIVE_CORRIDOR_2)
 *          no balls    → drive fallback path  (state = FALLBACK)
 *
 *   3. execute (after any drive finishes):    state = DONE
 *
 * Finishes when state is DONE, or when the spindexer is full (3-of-3
 * sensors), or when the follower finishes naturally.
 */
public class VisionCollectWithFallbacksCommand extends CommandBase {

    private final ArtifactDetector detector;
    private final Follower follower;
    private final double maxPower;
    private final double secondaryHeadingRad;
    private final PathChain fallbackPath;

    private enum State {
        SCAN_FIRST,
        DRIVE_CORRIDOR,
        ROTATE,
        DRIVE_CORRIDOR_2,
        FALLBACK,
        DONE
    }

    private State state = State.SCAN_FIRST;
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
        state = State.SCAN_FIRST;
        CorridorPlanner.Sweep sweep = CorridorSelector.selectCorridor(detector, follower.getPose());
        lastSweep = sweep;

        if (hasUsableSweep(sweep)) {
            driveCorridor(sweep);
            state = State.DRIVE_CORRIDOR;
            lastResult = "scan1 OK: " + corridorSummary(sweep);
        } else {
            driveRotateInPlace();
            state = State.ROTATE;
            lastResult = "scan1 EMPTY → rotating to " + Math.toDegrees(secondaryHeadingRad) + "°";
        }
    }

    @Override
    public void execute() {
        // Anything still driving? Wait for it.
        if (follower.isBusy()) return;

        switch (state) {
            case ROTATE:
                // Rotation finished; re-scan at the new heading.
                CorridorPlanner.Sweep sweep2 = CorridorSelector.selectCorridor(detector, follower.getPose());
                lastSweep = sweep2;

                if (hasUsableSweep(sweep2)) {
                    driveCorridor(sweep2);
                    state = State.DRIVE_CORRIDOR_2;
                    lastResult = "scan2 OK: " + corridorSummary(sweep2);
                } else {
                    follower.setMaxPower(maxPower);
                    follower.followPath(fallbackPath, false);
                    state = State.FALLBACK;
                    lastResult = "scan2 EMPTY → fallback path";
                }
                break;

            case DRIVE_CORRIDOR:
            case DRIVE_CORRIDOR_2:
            case FALLBACK:
                // Drive finished (follower idle) — done.
                state = State.DONE;
                break;

            case SCAN_FIRST:
            case DONE:
                // no-op
                break;
        }
    }

    @Override
    public boolean isFinished() {
        if (state == State.DONE) return true;

        // Intake full check only during drive states (not during rotation).
        if ((state == State.DRIVE_CORRIDOR
                || state == State.DRIVE_CORRIDOR_2
                || state == State.FALLBACK)
                && isFull()) {
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

    /**
     * Builds a near-zero-length BezierLine from the current pose to a pose
     * 0.001" ahead in the current heading's forward direction, with heading
     * interpolated to secondaryHeadingRad. Effectively an in-place rotation.
     */
    private void driveRotateInPlace() {
        Pose cur = follower.getPose();
        double eps = 0.001;
        Pose target = new Pose(
                cur.getX() + Math.cos(cur.getHeading()) * eps,
                cur.getY() + Math.sin(cur.getHeading()) * eps,
                secondaryHeadingRad);
        PathChain rotatePath = follower.pathBuilder()
                .addPath(new BezierLine(cur, target))
                .setLinearHeadingInterpolation(cur.getHeading(), secondaryHeadingRad)
                .build();
        follower.setMaxPower(maxPower);
        follower.followPath(rotatePath, false);
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
