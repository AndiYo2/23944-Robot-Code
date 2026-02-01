package Autos.NineBalls;

import Autos.AutonTemplate;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import commands.CommandSequenceBuilder;
@Autonomous(name = "Test", group = "NineBall")
public class Test extends AutonTemplate {
    private PathChain startToEnd;

    private final Pose startPose = new Pose(56.5, 8.5, Math.toRadians(90));

    private final Pose stopPose = new Pose(56.5, 72.0, Math.toRadians(90));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        startToEnd = follower.pathBuilder()
                .addPath(new BezierLine(startPose, stopPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), stopPose.getHeading())
                .setGlobalDeceleration()
                .build();

    }

    @Override
    public void init() {
        super.init();
        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .moveTo(startToEnd)
                .build();
    }
}
