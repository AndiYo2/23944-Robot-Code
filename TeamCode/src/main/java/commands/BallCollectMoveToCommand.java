package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import utility.RobotHardware;

/**
 * Follows a collect path until the robot detects 3 balls or the path ends.
 * This is the collect phase only — pair with {@link DynamicReturnPathCommand}
 * for the return trip.
 */
public class BallCollectMoveToCommand extends CommandBase {
    private final Follower follower;
    private final Object collectPath;
    private final double maxPower;

    public BallCollectMoveToCommand(Follower follower, Path collectPath, double maxPower) {
        this.follower = follower;
        this.collectPath = collectPath;
        this.maxPower = maxPower;
    }

    public BallCollectMoveToCommand(Follower follower, PathChain collectPath, double maxPower) {
        this.follower = follower;
        this.collectPath = collectPath;
        this.maxPower = maxPower;
    }

    @Override
    public void initialize() {
        follower.setMaxPower(maxPower);
        if (collectPath instanceof Path) {
            follower.followPath((Path) collectPath, false);
        } else {
            follower.followPath((PathChain) collectPath, false);
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        boolean pathDone = !follower.isBusy();
        boolean full = isFull();
        return pathDone || full;
    }

    @Override
    public void end(boolean interrupted) {
        follower.breakFollowing();
    }

    private boolean isFull() {
        RobotHardware hw = RobotHardware.getInstance();
        return hw.spindexerSensorPair.quickCheck().ballPresent
                && hw.rampSensorPair.quickCheck().ballPresent
                && hw.transferSensorPair.quickCheck().ballPresent;
    }
}