package Autos.Tests;

import Autos.AutonTemplate;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import commands.CommandSequenceBuilder;

@Autonomous(name = "SimplePathTest", group = "Autonomous")
public class SimplePathTest extends AutonTemplate {
    private PathChain path1, path2, path3, path4;

    private final Pose startPose = new Pose(72.000, 8.500, Math.toRadians(90));
    private final Pose pose1 = new Pose(72.000, 72.000, Math.toRadians(90));
    private final Pose pose2 = new Pose(64.000, 80.000, Math.toRadians(135));
    private final Pose pose3 = new Pose(29.500, 114.000, Math.toRadians(135));
    private final Pose pose4 = new Pose(105.500, 33.500, Math.toRadians(90));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), pose1.getHeading())
                .setGlobalDeceleration()
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(pose1, pose2))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .setGlobalDeceleration()
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(pose2, pose3))
                .setTangentHeadingInterpolation()
                .setGlobalDeceleration()
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(pose3, pose4))
                .setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading())
                .setGlobalDeceleration()
                .build();
    }

    @Override
    public void init() {
        super.init();

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .moveTo(path1)
                .delay(3)
                .moveTo(path2)
                .delay(3)
                .moveTo(path3)
                .delay(3)
                .moveTo(path4)
                .build();
    }
}
