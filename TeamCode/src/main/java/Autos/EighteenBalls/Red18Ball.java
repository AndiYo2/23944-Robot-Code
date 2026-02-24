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
    private PathChain startToShoot, Path15, shootToFirst, firstToShoot, shootToGatePrep1, prepToThree, gateToShoot1, shootToGatePrep2, Path13, GateToShoot2, ShootToFourth, fourthToShoot, Path16, shootToFifth, fifthToShootAndStop, Path14;

    // Named pose constants
    private final Pose startPose = new Pose(111.250, 135.000, Math.toRadians(90));
    private final Pose startingShootPose = new Pose(88.000, 80.000, Math.toRadians(90));

    // Path15 end pose
    private final Pose path15Pose = new Pose(98.000, 58.500, Math.toRadians(10));

    // ShootToFirst path
    private final Pose firstPickupPose = new Pose(123.500, 58.000, Math.toRadians(0));

    // Main shoot position
    private final Pose mainShootPose = new Pose(88.000, 82.000, Math.toRadians(45));

    // Gate paths
    private final Pose gatePrepControlPoint = new Pose(102.000, 68.500);
    private final Pose gateControlPoint = new Pose(108.000, 65.000);
    private final Pose gatePrepPose = new Pose(128.000, 63.000, Math.toRadians(0));
    private final Pose gateInnerControlPoint = new Pose(125.000, 57.500);
    private final Pose gatePose = new Pose(132.5, 55.000, Math.toRadians(10));
    private final Pose gateShootPose = new Pose(88.000, 80.000, Math.toRadians(30));
    private final Pose gateShootPose2 = new Pose(88.000, 82.000, Math.toRadians(30));

    // ShootToFourth path
    private final Pose fourthPickupPose = new Pose(127.000, 82.000, Math.toRadians(0));

    // Path16 path
    private final Pose path16ControlPoint = new Pose(82.500, 52.500);
    private final Pose path16Pose = new Pose(102.000, 33.000, Math.toRadians(0));

    // ShootToFifth path
    private final Pose fifthPickupPose = new Pose(133.000, 33.000, Math.toRadians(0));

    // Stop
    private final Pose lastShootPose = new Pose(89.500, 12.500, Math.toRadians(45));
    private final Pose stopPose = new Pose(89.000, 24.500, Math.toRadians(45));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, startingShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), startingShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        Path15 = follower.pathBuilder()
                .addPath(new BezierLine(startingShootPose, path15Pose))
                .setLinearHeadingInterpolation(startingShootPose.getHeading(), path15Pose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToFirst = follower.pathBuilder()
                .addPath(new BezierLine(path15Pose, firstPickupPose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), firstPickupPose.getHeading())
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

        Path13 = follower.pathBuilder()
                .addPath(new BezierCurve(gatePrepPose, gateInnerControlPoint, gatePose))
                .setLinearHeadingInterpolation(gatePrepPose.getHeading(), gatePose.getHeading())
                .setGlobalDeceleration()
                .build();

        GateToShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gateControlPoint, gateShootPose2))
                .setLinearHeadingInterpolation(gatePose.getHeading(), gateShootPose2.getHeading())
                .setGlobalDeceleration()
                .build();

        ShootToFourth = follower.pathBuilder()
                .addPath(new BezierLine(gateShootPose2, fourthPickupPose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setGlobalDeceleration()
                .build();

        fourthToShoot = follower.pathBuilder()
                .addPath(new BezierLine(fourthPickupPose, mainShootPose))
                .setLinearHeadingInterpolation(fourthPickupPose.getHeading(), mainShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        Path16 = follower.pathBuilder()
                .addPath(new BezierCurve(mainShootPose, path16ControlPoint, path16Pose))
                .setLinearHeadingInterpolation(mainShootPose.getHeading(), path16Pose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToFifth = follower.pathBuilder()
                .addPath(new BezierLine(path16Pose, fifthPickupPose))
                .setLinearHeadingInterpolation(path16Pose.getHeading(), fifthPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        fifthToShootAndStop = follower.pathBuilder()
                .addPath(new BezierLine(fifthPickupPose, lastShootPose))
                .setLinearHeadingInterpolation(fifthPickupPose.getHeading(), lastShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        Path14 = follower.pathBuilder()
                .addPath(new BezierLine(lastShootPose, stopPose))
                .setLinearHeadingInterpolation(lastShootPose.getHeading(), stopPose.getHeading())
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
                .parallel(p -> p.moveTo(startToShoot, maxSpeed, false).limelightScan())
                .shoot()
                .setShootWhileMoving(false)
                .moveTo(Path15, maxSpeed, false)
                .intakeStart()
                .moveTo(shootToFirst, maxSpeed, false)
                .parallel(p -> p.autoCatalog().moveTo(firstToShoot, maxSpeed, false))
                .shoot()
                .moveTo(shootToGatePrep1, .8, true)
                .delay(.4)
                .intakeStart()
                .moveTo(prepToThree, maxSpeed, true)
                .delay(1)
                .parallel(p -> p.autoCatalog().moveTo(gateToShoot1, maxSpeed, false))
                .shoot()
                .setSpindexerMode(EnumConstants.ShootingMode.Sorted)
                .delay(.5)
                .moveTo(shootToGatePrep2, .75, true)
                .delay(.015)
                .intakeStart()
                .moveTo(Path13, maxSpeed, true)
                .delay(1)
                .parallel(p -> p.autoCatalog().moveTo(GateToShoot2, maxSpeed, false))
                .shoot()
                .intakeStart()
                .moveTo(ShootToFourth, maxSpeed, false)
                .parallel(p -> p.autoCatalog().moveTo(fourthToShoot, maxSpeed, false))
                .shoot()
                .intakeStart()
                .moveTo(Path16, maxSpeed, false)
                .moveTo(shootToFifth, maxSpeed, false)
                .parallel(p -> p.moveTo(fifthToShootAndStop, maxSpeed, false).autoCatalog())
                .shoot()
                .moveTo(Path14, maxSpeed, false)
                .build();
    }
}