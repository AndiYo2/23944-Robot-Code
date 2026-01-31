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


@Autonomous(name = "Blue12Ball")
public class Blue12Ball extends AutonTemplate {
    public static double maxSpeed = .8;
    private PathChain shootToFirst, firstToGate, gateToShoot, shootToSecond, secondToShoot, shootToThird, thirdToShoot, shootToStop;

    // Named pose constants
    private final Pose startPose = new Pose(56.500, 8.500, Math.toRadians(90));
    private final Pose shootPose = new Pose(52.000, 12.500, Math.toRadians(90));

    // ShootToFirst path
    private final Pose firstControlPoint = new Pose(63.000, 64.000);
    private final Pose firstPickupPose = new Pose(18.500, 60, Math.toRadians(180));

    // FirstToGate path
    private final Pose gateControlPose = new Pose(24.500, 67.500);
    private final Pose gatePose = new Pose(17.000, 68.500, Math.toRadians(180));

    // GateToShoot path
    private final Pose gateToShootControlPoint = new Pose(59.500, 53.500);

    // ShootToSecond path
    private final Pose secondControlPoint = new Pose(46.500, 33.500);
    private final Pose secondPickupPose = new Pose(18.500, 36.500, Math.toRadians(180));

    // ShootToThird path
    private final Pose thirdControlPoint = new Pose(71.000, 87.000);
    private final Pose thirdPickupPose = new Pose(18.500, 85, Math.toRadians(180));

    // ShootToStop path
    private final Pose stopPose = new Pose(52.000, 32.500, Math.toRadians(90));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        shootToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, firstControlPoint, firstPickupPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        firstToGate = follower.pathBuilder()
                .addPath(new BezierCurve(firstPickupPose, gateControlPose, gatePose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), gatePose.getHeading())
                .setGlobalDeceleration()
                .build();

        gateToShoot = follower.pathBuilder()
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
                .addPath(new BezierCurve(shootPose, thirdControlPoint, thirdPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), thirdPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        thirdToShoot = follower.pathBuilder()
                .addPath(new BezierLine(thirdPickupPose, shootPose))
                .setLinearHeadingInterpolation(thirdPickupPose.getHeading(), shootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToStop = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, stopPose))
                .setTangentHeadingInterpolation()
                .setGlobalDeceleration()
                .build();
    }

    @Override
    public void init() {
        super.init();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Blue;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .delay(.2)
                .limelightScan()
                .shoot()
                .setSpindexerMode(EnumConstants.ShootingMode.Sorted)
                .intakeStart()
                .moveTo(shootToFirst, maxSpeed, true)
                .intakeStop()
                .moveTo(firstToGate, .5, true)
                .delay(.3)
                .parallel(p -> p.moveTo(gateToShoot, maxSpeed, true).catalog())
                .delay(.45)
                .shoot()
                .parallel(p -> p.moveTo(shootToSecond, maxSpeed, true).intakeStart())
                .intakeStop()
                .parallel(p -> p.moveTo(secondToShoot, maxSpeed, true).catalog())
                .delay(.45)
                .shoot()
                .parallel(p -> p.moveTo(shootToThird, maxSpeed, true).intakeStart())
                .delay(.3)
                .intakeStop()
                .parallel(p -> p.moveTo(thirdToShoot, maxSpeed, true).catalog())
                .delay(.45)
                .shoot()
                .moveTo(shootToStop, maxSpeed, true)
                .build();
    }
}