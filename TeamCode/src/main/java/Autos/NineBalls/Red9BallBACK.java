package Autos.NineBalls;

import Autos.AutonTemplate;
import Constants.EnumConstants;
import Constants.SpindexerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import commands.CommandSequenceBuilder;

@Disabled
@Autonomous(name = "Red9BallBACK")
public class Red9BallBACK extends AutonTemplate {
    public static double delayBeforeShootSecond = 5;
    public static double maxSpeed = .8;
    private PathChain shootToFirstOne, shootToFirstTwo, shootToFirstThree, firstToShoot, shootToSecond, shootToStop, shootToPark;

    // Named pose constants
    private final Pose startPose = new Pose(87.500, 8.500, Math.toRadians(90));
    private final Pose shootPose = new Pose(92.000, 12.500, Math.toRadians(65));

    // ShootToFirstOne path
    private final Pose firstOneControlPoint = new Pose(100.000, 19.500);
    private final Pose firstOnePose = new Pose(129.000, 12.750, Math.toRadians(345));

    // ShootToFirstTwo path
    private final Pose firstTwoPose = new Pose(130.700, 9.200, Math.toRadians(0));

    // ShootToFirstThree path
    private final Pose firstThreePose = new Pose(133.000, 9, Math.toRadians(0));

    // FirstToShoot path
    private final Pose firstToShootControlPoint = new Pose(111.500, 16.500);

    // ShootToSecond path
    private final Pose secondControlPoint = new Pose(94.858, 39.235);
    private final Pose secondPickupPose = new Pose(128.000, 35.000, Math.toRadians(0));

    // ShootToPark path
    private final Pose parkPose = new Pose(111.000, 11.500, Math.toRadians(0));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        shootToFirstOne = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, firstOneControlPoint, firstOnePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstOnePose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToFirstTwo = follower.pathBuilder()
                .addPath(new BezierLine(firstOnePose, firstTwoPose))
                .setLinearHeadingInterpolation(firstOnePose.getHeading(), firstTwoPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToFirstThree = follower.pathBuilder()
                .addPath(new BezierLine(firstTwoPose, firstThreePose))
                .setLinearHeadingInterpolation(firstTwoPose.getHeading(), firstThreePose.getHeading())
                .setGlobalDeceleration()
                .build();

        firstToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(firstThreePose, firstToShootControlPoint, shootPose))
                .setLinearHeadingInterpolation(firstThreePose.getHeading(), shootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, secondControlPoint, secondPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToStop = follower.pathBuilder()
                .addPath(new BezierLine(secondPickupPose, shootPose))
                .setLinearHeadingInterpolation(secondPickupPose.getHeading(), shootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .setGlobalDeceleration()
                .build();
    }

    @Override
    public void init() {
        super.init();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .delay(.2)
                .limelightScan()
                .shoot()
                .setSpindexerMode(EnumConstants.ShootingMode.Sorted)
                .intakeStart()
                .moveTo(shootToFirstOne, maxSpeed, true)
                .moveTo(shootToFirstTwo, maxSpeed, true)
                .moveTo(shootToFirstThree, maxSpeed, true)
                .intakeStop()
                .parallel(p -> p.moveTo(firstToShoot, maxSpeed, true).catalog())
                .delay(.3)
                .shoot()
                .parallel(p -> p.moveTo(shootToSecond, maxSpeed, true).intakeStart())
                .intakeStop()
                .parallel(p -> p.moveTo(shootToStop, maxSpeed, true).catalog())
                .delay(delayBeforeShootSecond)
                .shoot()
                .moveTo(shootToPark, maxSpeed, true)
                .build();
    }
}
