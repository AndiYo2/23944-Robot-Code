package Autos.Tests;

import Autos.AutonTemplate;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import commands.CommandSequenceBuilder;
@Disabled
@Autonomous(name = "testingAuton", group = "Autonomous")
public class testingAuton extends AutonTemplate {
    private PathChain path1;

    private final Pose startPose = new Pose(111.25, 132, Math.toRadians(90));
    private final Pose pose1 = new Pose(72.000, 72.000, Math.toRadians(90));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), pose1.getHeading())
                .setGlobalDeceleration()
                .build();


    }

    @Override
    public void init() {
        super.init();

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .moveTo(path1)
                .build();
    }
}
