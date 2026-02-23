package Autos.EighteenBalls;

import Autos.AutonTemplate;
import Constants.EnumConstants;
import Constants.SpindexerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;


@Autonomous(name = "Red18Ball")
public class Red18Ball extends AutonTemplate {
    public static double maxSpeed = 1;
    private PathChain startToShoot, shootToFirst, firstToShoot, shootToGatePrep1, prepToThree, gateToShoot1, shootToGatePrep2, path13, gateToShoot2, shootToFourth, fourthToShoot, shootToFifth, fifthToShootAndStop, testingStop;

    // Named pose constants
    private final Pose startPose = new Pose(111.250, 135.000, Math.toRadians(90));
    private final Pose shootPose = new Pose(88.000, 85.500, Math.toRadians(90));

    // ShootToFirst path
    private final Pose firstControlPoint = new Pose(78.000, 57.500);
    private final Pose firstPickupPose = new Pose(123.500, 55.500, Math.toRadians(0));

    // Main shoot position
    private final Pose mainShootPose = new Pose(86.000, 82.000, Math.toRadians(45));

    // Gate paths
    private final Pose gatePrepControlPoint = new Pose(102.000, 68.500);
    private final Pose gateControlPoint = new Pose(108.000, 65.000);
    private final Pose gatePrepPose = new Pose(128.000, 63.000, Math.toRadians(0));
    private final Pose gateInnerControlPoint = new Pose(125, 57.500);
    private final Pose gatePose = new Pose(132.500, 55.000, Math.toRadians(10));
    private final Pose gateShootPose = new Pose(86.000, 82.000, Math.toRadians(30));

    // ShootToFourth path
    private final Pose fourthPickupPose = new Pose(127.000, 82.000, Math.toRadians(0));

    // ShootToFifth path
    private final Pose fifthControlPoint = new Pose(85.000, 27.500);
    private final Pose fifthPickupPose = new Pose(125.000, 32.000, Math.toRadians(0));

    // Stop
    private final Pose stopPose = new Pose(92.000, 112.500, Math.toRadians(45));

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

        firstToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstPickupPose, gateShootPose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), gateShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToGatePrep1 = follower.pathBuilder()
                .addPath(new BezierCurve(gateShootPose, gatePrepControlPoint, gatePrepPose))
                .setLinearHeadingInterpolation(gateShootPose.getHeading(), gatePrepPose.getHeading())
                .setGlobalDeceleration()
                .build();

        prepToThree = follower.pathBuilder()
                .addPath(new BezierCurve(gatePrepPose, gateInnerControlPoint, gatePose))
                .setLinearHeadingInterpolation(gatePrepPose.getHeading(), gatePose.getHeading())
                .setGlobalDeceleration()
                .build();

        gateToShoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gateControlPoint, gateShootPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), gateShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToGatePrep2 = follower.pathBuilder()
                .addPath(new BezierCurve(gateShootPose, gatePrepControlPoint, gatePrepPose))
                .setLinearHeadingInterpolation(gateShootPose.getHeading(), gatePrepPose.getHeading())
                .setGlobalDeceleration()
                .build();

        path13 = follower.pathBuilder()
                .addPath(new BezierCurve(gatePrepPose, gateInnerControlPoint, gatePose))
                .setLinearHeadingInterpolation(gatePrepPose.getHeading(), gatePose.getHeading())
                .setGlobalDeceleration()
                .build();

        gateToShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gateControlPoint, gateShootPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), gateShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToFourth = follower.pathBuilder()
                .addPath(new BezierLine(gateShootPose, fourthPickupPose))
                .setLinearHeadingInterpolation(gateShootPose.getHeading(), fourthPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        fourthToShoot = follower.pathBuilder()
                .addPath(new BezierLine(fourthPickupPose, mainShootPose))
                .setLinearHeadingInterpolation(fourthPickupPose.getHeading(), mainShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToFifth = follower.pathBuilder()
                .addPath(new BezierCurve(mainShootPose, fifthControlPoint, fifthPickupPose))
                .setLinearHeadingInterpolation(mainShootPose.getHeading(), fifthPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        fifthToShootAndStop = follower.pathBuilder()
                .addPath(new BezierLine(fifthPickupPose, stopPose))
                .setLinearHeadingInterpolation(fifthPickupPose.getHeading(), stopPose.getHeading())
                .setGlobalDeceleration()
                .build();
        testingStop = follower.pathBuilder()
                .addPath(new BezierLine(stopPose, startPose))
                .setLinearHeadingInterpolation(stopPose.getHeading(), startPose.getHeading())
                .setGlobalDeceleration()
                .build();
    }

    @Override
    public void init() {
        super.init();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .setShootWhileMoving(true)
                .moveTo(startToShoot, maxSpeed, true)
                .shoot()
                .limelightScan()
                .setShootWhileMoving(false)
                .intakeStart()
                .moveTo(shootToFirst, maxSpeed, true)
                .autoCatalog()
                .moveTo(firstToShoot, maxSpeed, true)
                .shoot()
                .moveTo(shootToGatePrep1, .7, true)
                .delay(.02)
                .intakeStart()
                .moveTo(prepToThree, 1, true)
                .delay(1)
                .autoCatalog()
                .moveTo(gateToShoot1, maxSpeed, true)
                .shoot()
                .setSpindexerMode(EnumConstants.ShootingMode.Sorted)
                .delay(.5)
                .moveTo(shootToGatePrep2, .5, true)
                .delay(.02)
                .intakeStart()
                .moveTo(path13, 1, true)
                .delay(1)
                .autoCatalog()
                .moveTo(gateToShoot2, maxSpeed, true)
                .shoot()
                .intakeStart()
                .moveTo(shootToFourth, maxSpeed, true)
                .autoCatalog()
                .moveTo(fourthToShoot, maxSpeed, true)
                .shoot()
                .intakeStart()
                .moveTo(shootToFifth, maxSpeed, true)
                .autoCatalog()
                .moveTo(fifthToShootAndStop, maxSpeed, true)
                .shoot()
                .moveTo(testingStop, maxSpeed, true)
                .build();
    }
}