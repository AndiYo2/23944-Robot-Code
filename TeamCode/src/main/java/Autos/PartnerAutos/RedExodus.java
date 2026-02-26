package Autos.PartnerAutos;

import Autos.AutonTemplate;
import Constants.EnumConstants;
import Constants.SpindexerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;


@Autonomous(name = "RedExodus")
public class RedExodus extends AutonTemplate {
    public static double maxSpeed = 1;
    private PathChain startToShoot, shootToFirstPrep, firstPrepToFirst, firstToGate, firstGateToShoot, shootToGatePrep1, prepToThree, gateToShootThree, ShootToFourth, fourthToGate, fourthGateToShoot, shootToGatePrepFive, prepToFive, fiveToShoot;

    // Named pose constants
    private final Pose startPose = new Pose(112.000, 134.000, Math.toRadians(90));
    private final Pose startingShootPose = new Pose(88.000, 82.000, Math.toRadians(90));

    // ShootToFirstPrep path
    private final Pose firstPrepControlPoint = new Pose(90.000, 67.000);
    private final Pose firstPrepPose = new Pose(100.500, 59, Math.toRadians(0));

    // FirstPrepToFirst path
    private final Pose firstPickupPose = new Pose(123.500, 59, Math.toRadians(0));

    // FirstToGate path
    private final Pose openGateControlPoint = new Pose(120.000, 61.000);
    private final Pose gatePrepPose = new Pose(126.000, 66.000, Math.toRadians(0));

    // FirstGateToShoot path
    private final Pose firstGateControlPoint = new Pose(105.500, 58.500);
    private final Pose firstGatePrepPose = new Pose(128, 68.500, Math.toRadians(0));
    private final Pose gateShootPose = new Pose(88.000, 82.000, Math.toRadians(30));

    // Gate prep paths
    private final Pose gatePrepControlPoint = new Pose(102.000, 68.500);
    private final Pose gateInnerControlPoint = new Pose(125.000, 57.500);
    private final Pose gatePose = new Pose(132.500, 55.000, Math.toRadians(0));
    private final Pose gateControlPoint = new Pose(108.000, 65.000);

    // ShootToFourth path
    private final Pose fourthPickupPose = new Pose(125.000, 82.000, Math.toRadians(0));

    // FourthToGate path
    private final Pose fourthGateControlPoint = new Pose(120.000, 76.000);
    private final Pose fourthGatePose = new Pose(128.000, 75.000, Math.toRadians(0));

    // FiveToShoot path
    private final Pose lastShootPose = new Pose(86.000, 104.000, Math.toRadians(30));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, startingShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), startingShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToFirstPrep = follower.pathBuilder()
                .addPath(new BezierCurve(startingShootPose, firstPrepControlPoint, firstPrepPose))
                .setLinearHeadingInterpolation(startingShootPose.getHeading(), firstPrepPose.getHeading())
                .setGlobalDeceleration()
                .build();

        firstPrepToFirst = follower.pathBuilder()
                .addPath(new BezierLine(firstPrepPose, firstPickupPose))
                .setLinearHeadingInterpolation(firstPrepPose.getHeading(), firstPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        firstToGate = follower.pathBuilder()
                .addPath(new BezierCurve(firstPickupPose, openGateControlPoint, firstGatePrepPose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), firstGatePrepPose.getHeading())
                .setGlobalDeceleration()
                .build();

        firstGateToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(firstGatePrepPose, firstGateControlPoint, gateShootPose))
                .setLinearHeadingInterpolation(firstGatePrepPose.getHeading(), gateShootPose.getHeading())
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

        gateToShootThree = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gateControlPoint, gateShootPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), gateShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        ShootToFourth = follower.pathBuilder()
                .addPath(new BezierLine(gateShootPose, fourthPickupPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), fourthPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        fourthToGate = follower.pathBuilder()
                .addPath(new BezierCurve(fourthPickupPose, fourthGateControlPoint, fourthGatePose))
                .setLinearHeadingInterpolation(fourthPickupPose.getHeading(), fourthGatePose.getHeading())
                .setGlobalDeceleration()
                .build();

        fourthGateToShoot = follower.pathBuilder()
                .addPath(new BezierLine(fourthGatePose, gateShootPose))
                .setLinearHeadingInterpolation(fourthGatePose.getHeading(), gateShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToGatePrepFive = follower.pathBuilder()
                .addPath(new BezierCurve(gateShootPose, gatePrepControlPoint, gatePrepPose))
                .setLinearHeadingInterpolation(gateShootPose.getHeading(), gatePrepPose.getHeading())
                .setGlobalDeceleration()
                .build();

        prepToFive = follower.pathBuilder()
                .addPath(new BezierCurve(gatePrepPose, gateInnerControlPoint, gatePose))
                .setLinearHeadingInterpolation(gatePrepPose.getHeading(), gatePose.getHeading())
                .setGlobalDeceleration()
                .build();

        fiveToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gateControlPoint, lastShootPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), lastShootPose.getHeading())
                .setGlobalDeceleration()
                .build();
    }

    @Override
    public void init() {
        super.init();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .moveTo(startToShoot, maxSpeed, false)
                .shoot()
                .intakeStart()
                .moveTo(shootToFirstPrep, maxSpeed, false)
                .moveTo(firstPrepToFirst, maxSpeed, false)
                .intakeStop()
                .moveTo(firstToGate, .8, false)
                .delay(.8)
                .parallel(p -> p.autoCatalog().moveTo(firstGateToShoot, maxSpeed, false))
                .shoot()
                .moveTo(shootToGatePrep1, .8, true)
                .delay(1)
                .intakeStart()
                .moveTo(prepToThree, maxSpeed, false)
                .delay(.75)
                .intakeStop()
                .delay(1)
                .parallel(p -> p.autoCatalog().moveTo(gateToShootThree, maxSpeed, false))
                .shoot()
                .intakeStart()
                .moveTo(ShootToFourth, maxSpeed, false)
                .intakeStop()
                .moveTo(fourthToGate, maxSpeed, false)
                .delay(1)
                .parallel(p -> p.autoCatalog().moveTo(fourthGateToShoot, maxSpeed, false))
                .shoot()
                .moveTo(shootToGatePrepFive, .8, true)
                .delay(.75)
                .intakeStart()
                .moveTo(prepToFive, maxSpeed, false)
                .delay(1)
                .parallel(p -> p.autoCatalog().moveTo(fiveToShoot, maxSpeed, false))
                .shoot()
                .build();
    }
}