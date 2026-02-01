package Autos.TwelveBalls;

import Autos.AutonTemplate;
import Constants.EnumConstants;
import Constants.SpindexerConstants;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;

@Configurable
@Autonomous(name = "Blue9BallBACK")
public class Blue9BallBACK extends AutonTemplate {
    public static double delayBeforeShootSecond = 5;
    public static double maxSpeed = .8;
    private PathChain shootToFirstOne, shootToFirstTwo, shootToFirstThree, firstToShoot, shootToSecond, secondToShoot, shootToStop;

    // Named pose constants
    private final Pose startPose = new Pose(56.500, 8.500, Math.toRadians(90));
    private final Pose shootPose = new Pose(55.000, 15.000, Math.toRadians(90));

    // ShootToFirstOne path
    private final Pose firstOneControlPoint = new Pose(41.500, 20.000);
    private final Pose firstOnePose = new Pose(14.000, 12.750, Math.toRadians(195));

    // ShootToFirstTwo path
    private final Pose firstTwoPose = new Pose(14.000, 9.200, Math.toRadians(180));

    // ShootToFirstThree path
    private final Pose firstThreePose = new Pose(10.500, 9.000, Math.toRadians(180));

    // FirstToShoot path
    private final Pose firstToShootControlPoint = new Pose(30.500, 20.000);

    // ShootToSecond path
    private final Pose secondControlPoint = new Pose(53.000, 34.000);
    private final Pose secondPickupPose = new Pose(19.000, 36.000, Math.toRadians(180));

    // ShootToStop path
    private final Pose stopPose = new Pose(31.500, 15.000, Math.toRadians(90));

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
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Sorted;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Blue;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .delay(.2)
                .limelightScan()
                .catalog()
                .delay(.2)
                .shoot()
                .intakeStart()
                .moveTo(shootToFirstOne, maxSpeed, true)
                .moveTo(shootToFirstTwo, maxSpeed, true)
                .moveTo(shootToFirstThree, maxSpeed, true)
                .intakeStop()
                .parallel(p -> p.moveTo(firstToShoot, maxSpeed, true).catalog())
                .delay(.3)
                .shoot()
                .moveTo(shootToStop, maxSpeed, true)
                .build();
    }
}
