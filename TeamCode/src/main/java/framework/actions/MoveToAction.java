package framework.actions;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import framework.Action;

/**
 * Action that commands the robot to follow a path using the PedroPathing follower.
 * Completes when the follower reaches the end of the path.
 */
public class MoveToAction implements Action {
    private final Follower follower;
    private final Object path; // Can be Path or PathChain
    private final boolean holdEnd;
    private final double maxPower;
    private final String name;

    /**
     * Creates a MoveToAction with a Path.
     *
     * @param follower the follower to control
     * @param path the path to follow
     * @param holdEnd whether to hold position at the end of the path
     */
    public MoveToAction(Follower follower, Path path, boolean holdEnd) {
        this.follower = follower;
        this.path = path;
        this.maxPower = 1.0;
        this.holdEnd = holdEnd;
        this.name = "MoveTo[" + path.getClass().getSimpleName() + "]";
    }

    /**
     * Creates a MoveToAction with a PathChain.
     *
     * @param follower the follower to control
     * @param pathChain the path chain to follow
     * @param holdEnd whether to hold position at the end of the path
     */
    public MoveToAction(Follower follower, PathChain pathChain, boolean holdEnd) {
        this.follower = follower;
        this.path = pathChain;
        this.maxPower = 1.0;
        this.holdEnd = holdEnd;
        this.name = "MoveTo[PathChain]";
    }

    /**
     * Creates a MoveToAction with a Path and custom speed.
     *
     * @param follower the follower to control
     * @param path the path to follow
     * @param maxPower the maximum power/speed (0.0-1.0, where 0.5 = 50% speed)
     * @param holdEnd whether to hold position at the end of the path
     */
    public MoveToAction(Follower follower, Path path, double maxPower, boolean holdEnd) {
        this.follower = follower;
        this.path = path;
        this.maxPower = maxPower;
        this.holdEnd = holdEnd;
        this.name = "MoveTo[" + path.getClass().getSimpleName() + "]";
    }

    /**
     * Creates a MoveToAction with a PathChain and custom speed.
     *
     * @param follower the follower to control
     * @param pathChain the path chain to follow
     * @param maxPower the maximum power/speed (0.0-1.0)
     * @param holdEnd whether to hold position at the end of the path
     */
    public MoveToAction(Follower follower, PathChain pathChain, double maxPower, boolean holdEnd) {
        this.follower = follower;
        this.path = pathChain;
        this.maxPower = maxPower;
        this.holdEnd = holdEnd;
        this.name = "MoveTo[PathChain]";
    }

    @Override
    public void start() {
        follower.setMaxPower(maxPower);
        if (path instanceof Path) {
            follower.followPath((Path) path, holdEnd);
        } else if (path instanceof PathChain) {
            follower.followPath((PathChain) path, holdEnd);
        }
    }

    @Override
    public void update() {
        // Follower.update() is called in the main loop, no need to do anything here
    }

    @Override
    public boolean isComplete() {
        return !follower.isBusy();
    }

    @Override
    public void end() {
        // Nothing to clean up
    }

    @Override
    public String getName() {
        return name;
    }
}
