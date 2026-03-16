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


@Autonomous(name = "Red15Ball")
public class Red15Ball extends AutonTemplate {
    public static double maxSpeed = 1;
    private PathChain shootToFirst, firstToShoot, shootToSecond, secondToGate, gateToShoot, shootToThird, thirdToShoot, shootToFourth, fourthToShoot, shootToEnd;

    // Start pose
    private final Pose startPose = new Pose(88.000, 8.000, Math.toRadians(90));

    // First prep & pickup
    private final Pose shootToFirstPrepControl = new Pose(100.000, 17.000);
    private final Pose firstPrepPose = new Pose(119.500, 10.000, Math.toRadians(0));
    private final Pose firstPickupPose = new Pose(131.500, 10.000, Math.toRadians(0));

    // First to shoot
    private final Pose firstToShootControl = new Pose(109.000, 15.000);

    // Second prep & pickup
    private final Pose shootToSecondControl = new Pose(86.745, 52.766);
    private final Pose secondPrepPose = new Pose(99.000, 57.500, Math.toRadians(0));
    private final Pose secondPickupPose = new Pose(131.500, 57.500, Math.toRadians(0));

    // Gate area
    private final Pose secondToGateControl = new Pose(118.500, 62.500);
    private final Pose gatePose = new Pose(126.75, 65.500, Math.toRadians(0));

    // Gate to shoot
    private final Pose gateToShootControl = new Pose(96.500, 65.000);
    private final Pose gateShootPose = new Pose(86.000, 79.500, Math.toRadians(30));

    // Third pickup & main shoot area
    private final Pose postGateShootPose = new Pose(86.000, 79.500, Math.toRadians(0));
    private final Pose shootToThirdControl = new Pose(109.500, 78.000);
    private final Pose thirdPickupPose = new Pose(125.000, 84.000, Math.toRadians(0));

    // Fourth prep & pickup
    private final Pose shootToFourthPrepControl = new Pose(75.500, 35.500);
    private final Pose fourthPrepPose = new Pose(98.500, 36.000, Math.toRadians(0));
    private final Pose fourthPickupPose = new Pose(131.500, 36.000, Math.toRadians(0));

    // Shoot positions
    private final Pose firstShootPose = new Pose(88.500, 13.500, Math.toRadians(30));
    private final Pose cycleShootPose = new Pose(88.500, 12.000, Math.toRadians(30));


    // End pose
    private final Pose stopPose = new Pose(93.000, 23.000, Math.toRadians(30));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        shootToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, shootToFirstPrepControl, firstPrepPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstPrepPose.getHeading())
                .addPath(new BezierLine(firstPrepPose, firstPickupPose))
                .setLinearHeadingInterpolation(firstPrepPose.getHeading(), firstPickupPose.getHeading())
                .build();

        firstToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(firstPickupPose, firstToShootControl, firstShootPose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), firstShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(firstShootPose, shootToSecondControl, secondPrepPose))
                .setLinearHeadingInterpolation(firstShootPose.getHeading(), secondPrepPose.getHeading())
                .addPath(new BezierLine(secondPrepPose, secondPickupPose))
                .setTangentHeadingInterpolation()
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
                .addPath(new BezierCurve(postGateShootPose, shootToThirdControl, thirdPickupPose))
                .setLinearHeadingInterpolation(postGateShootPose.getHeading(), thirdPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        thirdToShoot = follower.pathBuilder()
                .addPath(new BezierLine(thirdPickupPose, postGateShootPose))
                .setLinearHeadingInterpolation(thirdPickupPose.getHeading(), postGateShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToFourth = follower.pathBuilder()
                .addPath(new BezierCurve(postGateShootPose, shootToFourthPrepControl, fourthPrepPose))
                .setLinearHeadingInterpolation(postGateShootPose.getHeading(), fourthPrepPose.getHeading())
                .addPath(new BezierLine(fourthPrepPose, fourthPickupPose))
                .setLinearHeadingInterpolation(fourthPrepPose.getHeading(), fourthPickupPose.getHeading())
                .build();

        fourthToShoot = follower.pathBuilder()
                .addPath(new BezierLine(fourthPickupPose, cycleShootPose))
                .setLinearHeadingInterpolation(fourthPickupPose.getHeading(), cycleShootPose.getHeading())
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

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .delay(.55)
                .parallel(p -> p.shoot().limelightScan())
                .intakeStart()
                .moveTo(shootToFirst, maxSpeed, false)
                .delay(1.1)
                .parallel(p -> p.moveTo(firstToShoot,maxSpeed,false).autoCatalog())
                .shoot()
                .intakeStart()
                .moveTo(shootToSecond, maxSpeed, false) //decrease Y and maybe more forwards
                .intakeStop()
                .moveTo(secondToGate, .8, false)
                .setSpindexerMode(EnumConstants.ShootingMode.Sorted)
                .delay(.6)
                .parallel(p -> p.moveTo(gateToShoot,maxSpeed,false).guaranteeSortedAutoCatalog())
                .slowShoot()
                .intakeStart()
                .moveTo(shootToThird, maxSpeed, false)
                .delay(.25)
                .parallel(p -> p.moveTo(thirdToShoot,maxSpeed,false).guaranteeSortedAutoCatalog())
                .slowShoot()
                .intakeStart()
                .moveTo(shootToFourth, maxSpeed, false)
                .delay(.125)
                .parallel(p -> p.moveTo(fourthToShoot,maxSpeed,false).guaranteeSortedAutoCatalog())
                .slowShoot()
                .moveTo(shootToEnd, maxSpeed, false)
                .build();
    }
}