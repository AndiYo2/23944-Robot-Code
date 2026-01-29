package Autos.TwelveBalls;

import Autos.AutonTemplate;
import Constants.EnumConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;


@Autonomous(name = "Red12Ball")
public class Red12Ball extends AutonTemplate {
    public static double maxSpeed = .8;
    private PathChain startToShoot, shootToFirst, firstToShoot, gateUnload, shootToSecond, secondToShoot, shootToThird, thirdToShoot, shootToStop;

    // Named pose constants
    private final Pose startPose = new Pose(87.500, 8.500, Math.toRadians(90));
    private final Pose shootPose = new Pose(92.000, 12.500, Math.toRadians(90));

    // ShootToFirst path
    private final Pose firstControlPoint = new Pose(90.000, 65.000);
    private final Pose firstPickupPose = new Pose(126.000, 57.500, Math.toRadians(0));

    // FirstToGate path
    private final Pose gateControlPose = new Pose(120.000, 69.000);
    private final Pose gatePose = new Pose(127.000, 67.000, Math.toRadians(0));

    // GateToShoot path
    private final Pose gateToShootControlPoint = new Pose(87.500, 52.500);

    // ShootToSecond path
    private final Pose secondControlPoint = new Pose(96.000, 31.000);
    private final Pose secondPickupPose = new Pose(126.000, 33.500, Math.toRadians(0));

    // ShootToThird paths (three segments)
    private final Pose thirdOnePose = new Pose(129.000, 12.750, Math.toRadians(345));
    private final Pose thirdTwoPose = new Pose(130.700, 9.200, Math.toRadians(0));
    private final Pose thirdPickupPose = new Pose(133.000, 9.000, Math.toRadians(0));

    // ThirdToShoot path
    private final Pose thirdToShootControlPoint = new Pose(115.000, 18.000);

    // ShootToStop path
    private final Pose stopPose = new Pose(92.000, 32.500, Math.toRadians(90));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, firstControlPoint, firstPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        gateUnload = follower.pathBuilder()
                .addPath(new BezierCurve(firstPickupPose, gateControlPose, gatePose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), gatePose.getHeading())
                .setGlobalDeceleration()
                .build();

        firstToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gateToShootControlPoint, shootPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose.getHeading())
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

        shootToThird = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, thirdOnePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), thirdOnePose.getHeading())
                .addPath(new BezierLine(thirdOnePose, thirdTwoPose))
                .setLinearHeadingInterpolation(thirdOnePose.getHeading(), thirdTwoPose.getHeading())
                .addPath(new BezierLine(thirdTwoPose, thirdPickupPose))
                .setLinearHeadingInterpolation(thirdTwoPose.getHeading(), thirdPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        thirdToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(thirdPickupPose, thirdToShootControlPoint, shootPose))
                .setLinearHeadingInterpolation(thirdPickupPose.getHeading(), shootPose.getHeading())
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
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Blue;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .limelightScan()
                .shoot()
                .setSpindexerMode(EnumConstants.ShootingMode.Sorted)
                .intakeStart()
                .moveTo(shootToFirst, maxSpeed)
                .intakeStop()
                .moveTo(gateUnload, maxSpeed)
                .delay(.3)
                .parallel(p -> p.moveTo(firstToShoot, maxSpeed).catalog())
                .shoot()
                .parallel(p -> p.moveTo(shootToSecond, maxSpeed).intakeStart())
                .parallel(p -> p.moveTo(secondToShoot, maxSpeed).catalog())
                .intakeStop()
                .shoot()
                .parallel(p -> p.moveTo(shootToThird, maxSpeed).intakeStart())
                .parallel(p -> p.moveTo(thirdToShoot, maxSpeed).catalog())
                .intakeStop()
                .shoot()
                .moveTo(shootToStop, maxSpeed)
                .build();
    }
}