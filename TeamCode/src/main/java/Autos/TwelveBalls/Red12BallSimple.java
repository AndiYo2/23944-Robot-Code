package Autos.TwelveBalls;

import Autos.AutonTemplate;
import Constants.EnumConstants;
import Constants.SpindexerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;


@Autonomous(name = "Red12BallSimple")
public class Red12BallSimple extends AutonTemplate {
    public static double maxSpeed = .8;
    private PathChain shootToFirst, firstToGate, gateToShoot, shootToSecond, secondToShoot, shootToThird, thirdToShoot, shootToStop;

    @Override
    protected void buildPaths() {
        follower.setStartingPose(new Pose(87.500, 8.500, Math.toRadians(90)));

        shootToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(87.500, 8.500),
                        new Pose(90.000, 65.000),
                        new Pose(126.000, 57.500)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        firstToGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(126.000, 57.500),
                        new Pose(120.000, 69.000),
                        new Pose(127.000, 67.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        gateToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(127.000, 67.000),
                        new Pose(87.500, 52.500),
                        new Pose(92.000, 12.500)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(92.000, 12.500),
                        new Pose(96.000, 31.000),
                        new Pose(126.000, 33.500)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        secondToShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(126.000, 33.500),
                        new Pose(92.000, 12.500)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        shootToThird = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(92.000, 12.500),
                        new Pose(83.000, 84.000),
                        new Pose(126.000, 83.000)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        thirdToShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(126.000, 83.000),
                        new Pose(92.000, 12.500)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        shootToStop = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(92.000, 12.500),
                        new Pose(92.000, 32.500)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
    }

    @Override
    public void init() {
        super.init();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .delay(.5)
                .limelightScan()
                .shoot()
                .setSpindexerMode(EnumConstants.ShootingMode.Sorted)
                .intakeStart()
                .moveTo(shootToFirst, maxSpeed, true)
                .intakeStop()
                .moveTo(firstToGate, .5, true)
                .delay(.3)
                .parallel(p -> p.moveTo(gateToShoot, maxSpeed, true).catalog())
                .shoot()
                .parallel(p -> p.moveTo(shootToSecond, maxSpeed, true).intakeStart())
                .intakeStop()
                .parallel(p -> p.moveTo(secondToShoot, maxSpeed, true).catalog())
                .shoot()
                .parallel(p -> p.moveTo(shootToThird, maxSpeed, true).intakeStart())
                .intakeStop()
                .parallel(p -> p.moveTo(thirdToShoot, maxSpeed, true).catalog())
                .shoot()
                .moveTo(shootToStop, maxSpeed, true)
                .build();
    }
}