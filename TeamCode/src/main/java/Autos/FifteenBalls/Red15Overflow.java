package Autos.FifteenBalls;

import Autos.AutonTemplate;
import Constants.EnumConstants;
import Constants.SpindexerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;


@Autonomous(name = "Red15Overflow")
public class Red15Overflow extends AutonTemplate {
    public static double maxSpeed = 1;
    private PathChain shootToFirstPrep, firstPrepToFirst, firstToShoot, shootToSecond, secondToGate, gateToShoot, shootToThird, thirdToShoot, shootToFourthPrep, fourthPrepToFourth, fourthToShoot, shootToCycle1Prep, cycle1PrepToCycle1, cycle1ToShoot, shootToEnd;

    // Start pose
    private final Pose startPose = new Pose(88.500, 8.500, Math.toRadians(90));

    // First prep & pickup
    private final Pose shootToFirstPrepControl = new Pose(100.500, 17.500);
    private final Pose firstPrepPose = new Pose(115.000, 9.000, Math.toRadians(0));
    private final Pose firstPickupPose = new Pose(133.500, 9.000, Math.toRadians(0));

    // Second pickup
    private final Pose shootToSecondControl = new Pose(81.500, 61.500);
    private final Pose secondPickupPose = new Pose(132.000, 58.000, Math.toRadians(0));

    // Gate area
    private final Pose secondToGateControl = new Pose(119.000, 63.000);
    private final Pose gatePose = new Pose(128.250, 64.000, Math.toRadians(0));

    // Gate to shoot
    private final Pose gateToShootControl = new Pose(96.000, 67.000);
    private final Pose gateShootPose = new Pose(86.500, 79.000, Math.toRadians(30));

    // Third pickup & main shoot area
    private final Pose postGateShootPose = new Pose(86.500, 79.000, Math.toRadians(0));
    private final Pose thirdPickupPose = new Pose(125.500, 79.000, Math.toRadians(0));

    // Fourth prep & pickup
    private final Pose shootToFourthPrepControl = new Pose(76.117, 36.043);
    private final Pose fourthPrepPose = new Pose(99.000, 31.000, Math.toRadians(0));
    private final Pose fourthPickupPose = new Pose(132.000, 31.000, Math.toRadians(0));

    // Cycle shoot position
    private final Pose cycleShootPose = new Pose(89.000, 12.500, Math.toRadians(30));

    // Cycle 1 pickup
    private final Pose cycle1PickupPose = new Pose(133.000, 9.000, Math.toRadians(0));

    // End pose
    private final Pose stopPose = new Pose(93.500, 23.500, Math.toRadians(30));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        shootToFirstPrep = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, shootToFirstPrepControl, firstPrepPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstPrepPose.getHeading())
                .setGlobalDeceleration()
                .build();

        firstPrepToFirst = follower.pathBuilder()
                .addPath(new BezierLine(firstPrepPose, firstPickupPose))
                .setLinearHeadingInterpolation(firstPrepPose.getHeading(), firstPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        firstToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstPickupPose, cycleShootPose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), cycleShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(cycleShootPose, shootToSecondControl, secondPickupPose))
                .setLinearHeadingInterpolation(cycleShootPose.getHeading(), secondPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        secondToGate = follower.pathBuilder()
                .addPath(new BezierCurve(secondPickupPose, secondToGateControl, gatePose))
                .setLinearHeadingInterpolation(secondPickupPose.getHeading(), gatePose.getHeading())
                .setGlobalDeceleration()
                .build();

        gateToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gateToShootControl, gateShootPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), gateShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToThird = follower.pathBuilder()
                .addPath(new BezierLine(postGateShootPose, thirdPickupPose))
                .setLinearHeadingInterpolation(postGateShootPose.getHeading(), thirdPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        thirdToShoot = follower.pathBuilder()
                .addPath(new BezierLine(thirdPickupPose, postGateShootPose))
                .setLinearHeadingInterpolation(thirdPickupPose.getHeading(), postGateShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToFourthPrep = follower.pathBuilder()
                .addPath(new BezierCurve(postGateShootPose, shootToFourthPrepControl, fourthPrepPose))
                .setLinearHeadingInterpolation(postGateShootPose.getHeading(), fourthPrepPose.getHeading())
                .setGlobalDeceleration()
                .build();

        fourthPrepToFourth = follower.pathBuilder()
                .addPath(new BezierLine(fourthPrepPose, fourthPickupPose))
                .setLinearHeadingInterpolation(fourthPrepPose.getHeading(), fourthPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        fourthToShoot = follower.pathBuilder()
                .addPath(new BezierLine(fourthPickupPose, cycleShootPose))
                .setLinearHeadingInterpolation(fourthPickupPose.getHeading(), cycleShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToCycle1Prep = follower.pathBuilder()
                .addPath(new BezierLine(cycleShootPose, firstPrepPose))
                .setLinearHeadingInterpolation(cycleShootPose.getHeading(), firstPrepPose.getHeading())
                .setGlobalDeceleration()
                .build();

        cycle1PrepToCycle1 = follower.pathBuilder()
                .addPath(new BezierLine(firstPrepPose, cycle1PickupPose))
                .setLinearHeadingInterpolation(firstPrepPose.getHeading(), cycle1PickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        cycle1ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(cycle1PickupPose, cycleShootPose))
                .setLinearHeadingInterpolation(cycle1PickupPose.getHeading(), cycleShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToEnd = follower.pathBuilder()
                .addPath(new BezierLine(cycleShootPose, stopPose))
                .setLinearHeadingInterpolation(cycleShootPose.getHeading(), stopPose.getHeading())
                .setGlobalDeceleration()
                .build();
    }

    @Override
    public void init() {
        super.init();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;
        Constants.TurretConstants.RED_TURRET_TRACKING_OFFSET -= 1;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .limelightScan()
                .delay(.5)
                .shoot()
                .intakeStart()
                .moveTo(shootToFirstPrep, maxSpeed, false)
                .moveTo(firstPrepToFirst, maxSpeed, false)
                .delay(.6)
                .parallel(p -> p.moveTo(firstToShoot,maxSpeed,false).autoCatalog())
                .shoot()
                .intakeStart()
                .moveTo(shootToSecond, maxSpeed, false) //decrease Y and maybe more forwards
                .intakeStop()
                .moveTo(secondToGate, .8, false)
                .setSpindexerMode(EnumConstants.ShootingMode.Sorted)
                .delay(.85)
                .parallel(p -> p.moveTo(gateToShoot,maxSpeed,false).autoCatalog())
                .shoot()
                .intakeStart()
                .moveTo(shootToThird, maxSpeed, false)
                .delay(.25)
                .parallel(p -> p.moveTo(thirdToShoot,maxSpeed,false).autoCatalog())
                .shoot()
                .intakeStart()
                .moveTo(shootToFourthPrep, maxSpeed, false)
                .moveTo(fourthPrepToFourth, maxSpeed, false)
                .delay(.25)
                .parallel(p -> p.moveTo(fourthToShoot,maxSpeed,false).autoCatalog())
                .shoot()
                .intakeStart()
                .setSpindexerMode(EnumConstants.ShootingMode.Fast)
                .moveTo(shootToCycle1Prep, maxSpeed, false)
                .moveTo(cycle1PrepToCycle1, maxSpeed, false)
                .parallel(p -> p.moveTo(cycle1ToShoot,maxSpeed,false).autoCatalog())
                .shoot()
                .moveTo(shootToEnd, maxSpeed, false)
                .build();
    }
}