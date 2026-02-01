package Autos.NineBalls;

import Autos.AutonTemplate;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import commands.CommandSequenceBuilder;
import Constants.EnumConstants;
import Constants.RobotConstants;
import utility.RobotHardware;
@Disabled
@Autonomous(name = "BlueFront9Ball", group = "NineBall")
public class BlueFront9Ball extends AutonTemplate {
    public static double maxSpeed = .8;
    private PathChain startToShoot, shootToFirst, firstToShoot, shootToSecond, secondToShoot, shootToStop;

    // Named pose constants (matching BlueBack9Ball pattern)
    private final Pose startPose = new Pose(21, 122.5, Math.toRadians(54));
    private final Pose shootPose = new Pose(54, 90, Math.toRadians(135));
    private final Pose firstPickupPose = new Pose(28, 86, Math.toRadians(180));
    private final Pose firstPickupControlPoint = new Pose(66.5, 78.5);
    private final Pose secondPickupPose = new Pose(28, 61, Math.toRadians(180));
    private final Pose secondPickupControlPoint = new Pose(69.7, 49.5);
    private final Pose stopPose = new Pose(32, 85, Math.toRadians(180));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, firstPickupControlPoint, firstPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        firstToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstPickupPose, shootPose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), shootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, secondPickupControlPoint, secondPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        secondToShoot = follower.pathBuilder()
                .addPath(new BezierLine(secondPickupPose, shootPose))
                .setLinearHeadingInterpolation(secondPickupPose.getHeading(), shootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToStop = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, stopPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), stopPose.getHeading())
                .setGlobalDeceleration()
                .build();
    }

    @Override
    public void init() {
        super.init();
        RobotConstants.Robot.allianceColor = EnumConstants.AllianceColor.Blue;
        RobotHardware.getInstance().limelight.pause();

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .parallel(p -> p.moveTo(startToShoot, maxSpeed).catalog())
                .limelightScan()
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