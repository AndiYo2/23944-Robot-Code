package Autos.Tests;

import Autos.AutonTemplate;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import framework.AutonSequence;

@Autonomous(name = "SimpleTurnTest", group = "Autonomous")
public class SimpleTurnTest extends AutonTemplate {
    private PathChain path1, path2, path3, path4;

    private final Pose startPose = new Pose(72.000, 8.500, Math.toRadians(90));
    private final Pose pose1 = new Pose(72.000, 72.000, Math.toRadians(180));
    private final Pose pose2 = new Pose(96.000, 96.000, Math.toRadians(0));
    private final Pose pose3 = new Pose(48, 120, Math.toRadians(270));
    private final Pose pose4 = new Pose(105.500, 33.500, Math.toRadians(90));

    private final Pose controlPoint = new Pose(2,23,Math.toRadians(0));

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
                .addPath(new BezierCurve(pose3,controlPoint, pose4))
                .setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading())
                .setGlobalDeceleration()
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
    }

    @Override
    public void init() {
        super.init();

        executor = new AutonSequence(follower, intake, spindexerManager, limelight)
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
