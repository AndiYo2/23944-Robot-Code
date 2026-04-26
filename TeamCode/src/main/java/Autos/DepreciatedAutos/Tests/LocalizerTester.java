package Autos.DepreciatedAutos.Tests;

import Autos.AutonTemplate;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import commands.CommandSequenceBuilder;
@Disabled
@Autonomous(name = "LocalizerTester", group = "Autonomous")
public class LocalizerTester extends AutonTemplate {
    private PathChain path1, path2, path3, path4, path5, path6;

    private final Pose startPose = new Pose(72.000, 72.000, Math.toRadians(90));
    private final Pose pose1Control = new Pose(8.250, 104.500);
    private final Pose pose1 = new Pose(80.500, 129.000, Math.toRadians(90));
    private final Pose pose2Control = new Pose(124.000, 112.000);
    private final Pose pose2 = new Pose(105.000, 56.500, Math.toRadians(180));
    private final Pose pose3 = new Pose(33.000, 104.500, Math.toRadians(0));
    private final Pose pose4Control1 = new Pose(19.500, 60.500);
    private final Pose pose4Control2 = new Pose(103.000, 87.500);
    private final Pose pose4 = new Pose(80.500, 15.000, Math.toRadians(270));
    private final Pose pose5Control1 = new Pose(23.500, 29.500);
    private final Pose pose5Control2 = new Pose(85.000, 57.000);
    private final Pose pose5 = new Pose(80.500, 105.000, Math.toRadians(90));
    private final Pose pose6 = new Pose(105.500, 33.000, Math.toRadians(0));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        path1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, pose1Control, pose1))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(pose1, pose2Control, pose2))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(pose2, pose3))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(pose3, pose4Control1, pose4Control2, pose4))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierCurve(pose4, pose5Control1, pose5Control2, pose5))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(90))
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(pose5, pose6))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();
    }

    @Override
    public void init() {
        super.init();

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .moveTo(path1)
                .delay(4)
                .moveTo(path2)
                .delay(4)
                .moveTo(path3)
                .delay(4)
                .moveTo(path4)
                .delay(4)
                .moveTo(path5)
                .delay(4)
                .moveTo(path6)
                .build();
    }
}
