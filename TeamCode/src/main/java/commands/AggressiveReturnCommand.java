package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/**
 * Like {@link DynamicReturnPathCommand} but builds the path with overridden
 * braking constants so short returns don't spend the whole path in the
 * deceleration zone. Used by the defense anchor feature in TeleOp — robot is
 * pushed off a saved pose, follower drives back at max power with delayed /
 * stronger braking for a snappy return.
 */
public class AggressiveReturnCommand extends CommandBase {
    private final Follower follower;
    private final Pose target;
    private final double maxPower;
    private final double brakingStart;
    private final double brakingStrength;

    public AggressiveReturnCommand(Follower follower, Pose target, double maxPower,
                                   double brakingStart, double brakingStrength) {
        this.follower = follower;
        this.target = target;
        this.maxPower = maxPower;
        this.brakingStart = brakingStart;
        this.brakingStrength = brakingStrength;
    }

    @Override
    public void initialize() {
        Pose start = follower.getPose();
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, target))
                .setLinearHeadingInterpolation(start.getHeading(), target.getHeading())
                .setBrakingStart(brakingStart)
                .setBrakingStrength(brakingStrength)
                .build();
        follower.setMaxPower(maxPower);
        follower.followPath(path, false);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) follower.breakFollowing();
    }
}
