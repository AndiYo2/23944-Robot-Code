package Autos.NineBalls;

import Autos.AutonTemplate;
import Constants.EnumConstants;
import Constants.SpindexerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;


@Autonomous(name = "Red9BallFRONT")
public class Red9BallFRONT extends AutonTemplate {
    public static double delayBeforeShootSecond = 5;
    public static double maxSpeed = .8;
    private PathChain startToScan, scanToShoot, shootToSecond, secondToShoot, shootToFirst, firstToShoot, shootToStop;

    // Named pose constants
    private final Pose startPose = new Pose(128.500, 113.500, Math.toRadians(180));
    private final Pose scanPose = new Pose(98.500, 97.500, Math.toRadians(90));
    private final Pose shootPose = new Pose(90.000, 90.000, Math.toRadians(45));

    // ShootToSecond path
    private final Pose secondControlPoint = new Pose(82.000, 55.000);
    private final Pose secondPickupPose = new Pose(127.000, 56.000, Math.toRadians(0));

    // SecondToShoot path
    private final Pose secondToShootControlPoint = new Pose(88.000, 69.500);

    // ShootToFirst path
    private final Pose firstControlPoint = new Pose(85.500, 76.298);
    private final Pose firstPickupPose = new Pose(128.500, 82, Math.toRadians(0));

    // ShootToStop path
    private final Pose stopPose = new Pose(116.000, 72.000, Math.toRadians(0));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        startToScan = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scanPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading())
                .setGlobalDeceleration()
                .build();

        scanToShoot = follower.pathBuilder()
                .addPath(new BezierLine(scanPose, shootPose))
                .setLinearHeadingInterpolation(scanPose.getHeading(), shootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, secondControlPoint, secondPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        secondToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(secondPickupPose, secondToShootControlPoint, shootPose))
                .setLinearHeadingInterpolation(secondPickupPose.getHeading(), shootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, firstControlPoint, firstPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        firstToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstPickupPose, shootPose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), shootPose.getHeading())
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
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Sorted;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .delay(.2)
                .moveTo(startToScan, maxSpeed, true)
                .delay(.2)
                .limelightScan()
                .moveTo(scanToShoot, maxSpeed, true)
                .catalog()
                .shoot()
                .parallel(p -> p.moveTo(shootToSecond, maxSpeed, true).intakeStart())
                .intakeStop()
                .parallel(p -> p.moveTo(secondToShoot, maxSpeed, true).catalog())
                .delay(.3)
                .shoot()
                .parallel(p -> p.moveTo(shootToFirst, maxSpeed, true).intakeStart())
                .intakeStop()
                .parallel(p -> p.moveTo(firstToShoot, maxSpeed, true).catalog())
                .delay(.3)
                .shoot()
                .moveTo(shootToStop, maxSpeed, true)
                .build();
    }
}
