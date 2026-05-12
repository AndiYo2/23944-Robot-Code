package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import utility.RobotHardware;

/**
 * Follows a collect path until 3 balls have been collected.
 *
 * Tracks rising edges on each distance sensor every loop. As a ball passes ramp
 * → transfer → spindexer, each sensor logs one rising edge per ball — even if
 * the spindexer rotates the ball away (which makes a "simultaneous all-3-high"
 * check impossible to satisfy in normal operation). Exits on the FIRST of:
 *   1. Any sensor's edge count >= 3.
 *   2. All 3 sensors high in the same loop (pipeline backed up).
 *   3. maxWaitAfterPathSec elapsed after path completion.
 *
 * Distance presence is preferred over quickCheck().ballPresent because the
 * latter requires color identification, gated by a 5-cycle color burst polled
 * one pair per loop. Distance reads come from the REV bulk cache and refresh
 * every loop.
 */
public class BallCollectMoveToCommand extends CommandBase {
    public static volatile String lastDebug = "idle";

    private static final double DEFAULT_MAX_WAIT_AFTER_PATH_SEC = 0.4;
    private static final int TARGET_BALL_COUNT = 3;

    private final Follower follower;
    private final Object collectPath;
    private final double maxPower;
    private final double maxWaitAfterPathSec;
    private final boolean taperEnabled;
    private final double decelStartT;
    private final double endPower;

    private final ElapsedTime timer = new ElapsedTime();
    private boolean pathDoneRecorded = false;
    private double pathDoneTime = 0.0;

    private boolean lastSpindexer = false;
    private boolean lastRamp = false;
    private boolean lastTransfer = false;
    private int spindexerEdges = 0;
    private int rampEdges = 0;
    private int transferEdges = 0;
    private boolean allHighNow = false;

    public BallCollectMoveToCommand(Follower follower, Path collectPath, double maxPower) {
        this(follower, (Object) collectPath, maxPower, DEFAULT_MAX_WAIT_AFTER_PATH_SEC, false, 1.0, maxPower);
    }

    public BallCollectMoveToCommand(Follower follower, PathChain collectPath, double maxPower) {
        this(follower, (Object) collectPath, maxPower, DEFAULT_MAX_WAIT_AFTER_PATH_SEC, false, 1.0, maxPower);
    }
    public BallCollectMoveToCommand(Follower follower, Path collectPath, double maxPower,
                                    double decelStartT, double endPower) {
        this(follower, (Object) collectPath, maxPower, DEFAULT_MAX_WAIT_AFTER_PATH_SEC, true, decelStartT, endPower);
    }

    public BallCollectMoveToCommand(Follower follower, PathChain collectPath, double maxPower,
                                    double decelStartT, double endPower) {
        this(follower, (Object) collectPath, maxPower, DEFAULT_MAX_WAIT_AFTER_PATH_SEC, true, decelStartT, endPower);
    }
    private BallCollectMoveToCommand(Follower follower, Object collectPath, double maxPower,
                                     double maxWaitAfterPathSec, boolean taperEnabled,
                                     double decelStartT, double endPower) {
        this.follower = follower;
        this.collectPath = collectPath;
        this.maxPower = maxPower;
        this.maxWaitAfterPathSec = maxWaitAfterPathSec;
        this.taperEnabled = taperEnabled;
        this.decelStartT = Math.max(0.0, Math.min(1.0, decelStartT));
        this.endPower = endPower;
    }

    @Override
    public void initialize() {
        pathDoneRecorded = false;
        pathDoneTime = 0.0;
        timer.reset();

        lastSpindexer = false;
        lastRamp = false;
        lastTransfer = false;
        spindexerEdges = 0;
        rampEdges = 0;
        transferEdges = 0;
        allHighNow = false;

        follower.setMaxPower(maxPower);
        if (collectPath instanceof Path) {
            follower.followPath((Path) collectPath, true);
        } else {
            follower.followPath((PathChain) collectPath, true);
        }
    }

    @Override
    public void execute() {
        if (taperEnabled && follower.getCurrentPath() != null && follower.isBusy()) {
            double tProgress = follower.getCurrentPath().getClosestPointTValue();
            double power;
            if (tProgress <= decelStartT || decelStartT >= 1.0) {
                power = maxPower;
            } else {
                double frac = (tProgress - decelStartT) / (1.0 - decelStartT);
                if (frac > 1.0) frac = 1.0;
                power = maxPower + (endPower - maxPower) * frac;
            }
            follower.setMaxPower(power);
        }

        RobotHardware hw = RobotHardware.getInstance();
        boolean s = hw.spindexerSensorPair.checkDistancePresent();
        boolean r = hw.rampSensorPair.checkDistancePresent();
        boolean t = hw.transferSensorPair.checkDistancePresent();

        if (s && !lastSpindexer) spindexerEdges++;
        if (r && !lastRamp) rampEdges++;
        if (t && !lastTransfer) transferEdges++;

        lastSpindexer = s;
        lastRamp = r;
        lastTransfer = t;
        allHighNow = s && r && t;

        lastDebug = String.format(
                "spin=%s(%d) ramp=%s(%d) xfer=%s(%d) pathBusy=%s t=%.2f",
                s ? "T" : "F", spindexerEdges,
                r ? "T" : "F", rampEdges,
                t ? "T" : "F", transferEdges,
                follower.isBusy(), timer.seconds());
    }

    @Override
    public boolean isFinished() {
        if (allHighNow) {
            return true;
        }
        int maxEdges = Math.max(spindexerEdges, Math.max(rampEdges, transferEdges));
        if (maxEdges >= TARGET_BALL_COUNT) {
            return true;
        }

        if (!follower.isBusy()) {
            if (!pathDoneRecorded) {
                pathDoneRecorded = true;
                pathDoneTime = timer.seconds();
            }
            if (timer.seconds() - pathDoneTime >= maxWaitAfterPathSec) {
                return true;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        follower.breakFollowing();
    }
}
