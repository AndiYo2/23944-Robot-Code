package Autos.DepreciatedAutos.Tests;

import Autos.AutonTemplate;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import commands.CommandSequenceBuilder;

@Disabled
@Autonomous(name = "21BallTesting")
public class BallTesting21 extends AutonTemplate {
    public static double maxSpeed = 1;

    // Start pose
    public static double startX = 88.500;
    public static double startY = 6.750;
    public static double startHeading = 90;

    // Mid pose
    public static double midX = 106.500;
    public static double midY = 21.500;
    public static double midHeading = 45;

    // End pose
    public static double endX = 122.500;
    public static double endY = 27.500;
    public static double endHeading = 45;

    // Path3 end pose
    public static double path3EndX = 89.500;
    public static double path3EndY = 14.000;
    public static double path3EndHeading = 45;

    // Control point
    public static double path2ControlX = 112.000;
    public static double path2ControlY = 27.500;

    // Second cycle mid pose
    public static double secondMidX = 106.500;
    public static double secondMidY = 45.500;
    public static double secondMidHeading = 45;

    // Second cycle end pose
    public static double secondEndX = 122.500;
    public static double secondEndY = 51.500;
    public static double secondEndHeading = 45;

    // Second cycle return end pose
    public static double secondReturnEndX = 89.500;
    public static double secondReturnEndY = 14.000;
    public static double secondReturnEndHeading = 45;

    // Second cycle control point
    public static double secondControlX = 112.000;
    public static double secondControlY = 51.500;

    private PathChain thirdSpikeToShootEnd, path3, secondCycle, secondReturn;

    @Override
    protected void buildPaths() {
        Pose startPose = new Pose(startX, startY, Math.toRadians(startHeading));
        Pose midPose = new Pose(midX, midY, Math.toRadians(midHeading));
        Pose endPose = new Pose(endX, endY, Math.toRadians(endHeading));
        Pose path3EndPose = new Pose(path3EndX, path3EndY, Math.toRadians(path3EndHeading));
        Pose path2Control = new Pose(path2ControlX, path2ControlY);

        follower.setStartingPose(startPose);

        thirdSpikeToShootEnd = follower.pathBuilder()
                .addPath(new BezierLine(startPose, midPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), midPose.getHeading())
                .addPath(new BezierCurve(midPose, path2Control, endPose))
                .setLinearHeadingInterpolation(midPose.getHeading(), endPose.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(endPose, path3EndPose))
                .setLinearHeadingInterpolation(endPose.getHeading(), path3EndPose.getHeading())
                .build();

        Pose secondMidPose = new Pose(secondMidX, secondMidY, Math.toRadians(secondMidHeading));
        Pose secondEndPose = new Pose(secondEndX, secondEndY, Math.toRadians(secondEndHeading));
        Pose secondReturnEndPose = new Pose(secondReturnEndX, secondReturnEndY, Math.toRadians(secondReturnEndHeading));
        Pose secondControl = new Pose(secondControlX, secondControlY);

        secondCycle = follower.pathBuilder()
                .addPath(new BezierLine(path3EndPose, secondMidPose))
                .setLinearHeadingInterpolation(path3EndPose.getHeading(), secondMidPose.getHeading())
                .addPath(new BezierCurve(secondMidPose, secondControl, secondEndPose))
                .setLinearHeadingInterpolation(secondMidPose.getHeading(), secondEndPose.getHeading())
                .build();

        secondReturn = follower.pathBuilder()
                .addPath(new BezierLine(secondEndPose, secondReturnEndPose))
                .setLinearHeadingInterpolation(secondEndPose.getHeading(), secondReturnEndPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        super.init();
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;
        maxSpeed = 1;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .intakeStart()
                .moveTo(thirdSpikeToShootEnd, maxSpeed, false)
                .parallel(p -> p.moveTo(path3, maxSpeed, false).autoCatalog())
                .shoot()
                .intakeStart()
                .moveTo(secondCycle, maxSpeed, false)
                .parallel(p -> p.moveTo(secondReturn, maxSpeed, false).autoCatalog())
                .shoot()
                .build();
    }
}
