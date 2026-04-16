package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

import java.util.function.DoubleUnaryOperator;

/**
 * Follows a path with a speed that varies based on parametric progress (t-value).
 * Accepts a callback that maps t (0.0-1.0) to maxPower (0.0-1.0).
 */
public class ParametricSpeedFollowCommand extends CommandBase {
    private final Follower follower;
    private final PathChain pathChain;
    private final boolean holdEnd;
    private final DoubleUnaryOperator speedFunction;

    /**
     * @param follower      the path follower
     * @param pathChain     the path to follow
     * @param speedFunction maps t-value (0.0-1.0) to maxPower (0.0-1.0)
     * @param holdEnd       whether to hold position at path end
     */
    public ParametricSpeedFollowCommand(Follower follower, PathChain pathChain,
                                         DoubleUnaryOperator speedFunction, boolean holdEnd) {
        this.follower = follower;
        this.pathChain = pathChain;
        this.speedFunction = speedFunction;
        this.holdEnd = holdEnd;
    }

    @Override
    public void initialize() {
        follower.setMaxPower(speedFunction.applyAsDouble(0.0));
        follower.followPath(pathChain, holdEnd);
    }

    @Override
    public void execute() {
        if (follower.getCurrentPath() != null) {
            double t = follower.getCurrentPath().getClosestPointTValue();
            follower.setMaxPower(speedFunction.applyAsDouble(t));
        }
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            follower.breakFollowing();
        }
    }
}