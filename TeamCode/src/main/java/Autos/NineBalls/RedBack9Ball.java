package Autos.NineBalls;

import Autos.AutonTemplate;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;
import Constants.EnumConstants;
import Constants.RobotConstants;

@Autonomous(name = "RedBack9Ball", group = "NineBall")
public class RedBack9Ball extends AutonTemplate {
    public static double maxSpeed = .8;
    private PathChain shootToFirst, firstToShoot, shootToSecond, secondToShoot, shootToStop;

    private final Pose startPose = new Pose(87.5, 8.5, Math.toRadians(90));
    private final Pose firstPickupPose = new Pose(123.5, 33.0, Math.toRadians(0));
    private final Pose firstPickupControlPoint = new Pose(90.0, 30.5);
    private final Pose firstShootPose = new Pose(89.0, 15.0, Math.toRadians(90));
    private final Pose secondPickupPose = new Pose(123.5, 60.0, Math.toRadians(0));
    private final Pose secondPickupControlPoint = new Pose(81.5, 51.6);
    private final Pose secondShootPose = new Pose(89.0, 15.0, Math.toRadians(90));
    private final Pose stopPose = new Pose(87.0, 45.0, Math.toRadians(90));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        shootToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, firstPickupControlPoint, firstPickupPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        firstToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstPickupPose, firstShootPose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), firstShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(firstShootPose, secondPickupControlPoint, secondPickupPose))
                .setLinearHeadingInterpolation(firstShootPose.getHeading(), secondPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        secondToShoot = follower.pathBuilder()
                .addPath(new BezierLine(secondPickupPose, secondShootPose))
                .setLinearHeadingInterpolation(secondPickupPose.getHeading(), secondShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToStop = follower.pathBuilder()
                .addPath(new BezierLine(secondShootPose, stopPose))
                .setLinearHeadingInterpolation(secondShootPose.getHeading(), stopPose.getHeading())
                .setGlobalDeceleration()
                .build();
    }

    @Override
    public void init() {
        super.init();
        RobotConstants.Robot.allianceColor = EnumConstants.AllianceColor.Red;
        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .parallel(p -> p.limelightScan().catalog())
                .shoot()
                .parallel(p -> p.moveTo(shootToFirst, maxSpeed).intakeStart())
                .intakeStop()
                .parallel(p -> p.moveTo(firstToShoot, maxSpeed).catalog())
                .shoot()
                .parallel(p -> p.moveTo(shootToSecond, maxSpeed).intakeStart())
                .intakeStop()
                .parallel(p -> p.moveTo(secondToShoot, maxSpeed).catalog())
                .shoot()
                .moveTo(shootToStop, maxSpeed)
                .build();
    }
}
