package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

/**
 * Command that follows a path using the PedroPathing follower.
 * Completes when the follower reaches the end of the path.
 */
public class FollowPathCommand extends CommandBase {
    private final Follower follower;
    private final Object path; // Can be Path or PathChain
    private final boolean holdEnd;
    private final double maxPower;

    /**
     * Creates a FollowPathCommand with a Path.
     *
     * @param follower the follower to control
     * @param path the path to follow
     * @param holdEnd whether to hold position at the end of the path
     */
    public FollowPathCommand(Follower follower, Path path, boolean holdEnd) {
        this(follower, path, 1.0, holdEnd);
    }

    /**
     * Creates a FollowPathCommand with a PathChain.
     *
     * @param follower the follower to control
     * @param pathChain the path chain to follow
     * @param holdEnd whether to hold position at the end of the path
     */
    public FollowPathCommand(Follower follower, PathChain pathChain, boolean holdEnd) {
        this(follower, pathChain, 1.0, holdEnd);
    }

    /**
     * Creates a FollowPathCommand with a Path and custom speed.
     *
     * @param follower the follower to control
     * @param path the path to follow
     * @param maxPower the maximum power/speed (0.0-1.0)
     * @param holdEnd whether to hold position at the end of the path
     */
    public FollowPathCommand(Follower follower, Path path, double maxPower, boolean holdEnd) {
        this.follower = follower;
        this.path = path;
        this.maxPower = maxPower;
        this.holdEnd = holdEnd;
        // No subsystem requirements - Follower operates independently
    }

    /**
     * Creates a FollowPathCommand with a PathChain and custom speed.
     *
     * @param follower the follower to control
     * @param pathChain the path chain to follow
     * @param maxPower the maximum power/speed (0.0-1.0)
     * @param holdEnd whether to hold position at the end of the path
     */
    public FollowPathCommand(Follower follower, PathChain pathChain, double maxPower, boolean holdEnd) {
        this.follower = follower;
        this.path = pathChain;
        this.maxPower = maxPower;
        this.holdEnd = holdEnd;
        // No subsystem requirements - Follower operates independently
    }

    @Override
    public void initialize() {
        follower.setMaxPower(maxPower);
        if (path instanceof Path) {
            follower.followPath((Path) path, holdEnd);
        } else if (path instanceof PathChain) {
            follower.followPath((PathChain) path, holdEnd);
        }
    }

    @Override
    public void execute() {
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