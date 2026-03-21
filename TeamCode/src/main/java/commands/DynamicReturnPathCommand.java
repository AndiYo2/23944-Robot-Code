package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/**
 * Dynamically builds a straight-line return path from the robot's current position
 * to a target pose when initialized. The path is built in initialize() so the
 * follower properly registers it as busy.
 */
public class DynamicReturnPathCommand extends CommandBase {
    private final Follower follower;
    private final Pose returnPose;
    private final double maxPower;
    private final boolean holdEnd;

    public DynamicReturnPathCommand(Follower follower, Pose returnPose, double maxPower, boolean holdEnd) {
        this.follower = follower;
        this.returnPose = returnPose;
        this.maxPower = maxPower;
        this.holdEnd = holdEnd;
    }

    @Override
    public void initialize() {
        Pose currentPose = follower.getPose();
        PathChain returnPath = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, returnPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), returnPose.getHeading())
                .build();
        follower.setMaxPower(maxPower);
        follower.followPath(returnPath, holdEnd);
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