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


@Autonomous(name = "\"Working\"Red18Ball")
public class Red18Ball extends AutonTemplate {
    public static double maxSpeed = 1;
    private PathChain startToShoot, shootToFirst, firstToShoot, ShootToGate1, GateToShoot1, ShootToGate2, GateToShoot2, ShootToFourth, fourthToShoot, shootToFifth, fifthToShootAndStop, testingStop;

    // Named pose constants
    private final Pose startPose = new Pose(111.2500, 132.00, Math.toRadians(90)); //DO NOT CHANGE EVEN IF THE POINTS I PASS IN ARE DIFFERENT
    private final Pose shootPose = new Pose(88.000, 82.500, Math.toRadians(45));

    // ShootToFirst path
    private final Pose firstControlPoint = new Pose(78, 54.5);
    private final Pose firstPickupPose = new Pose(123.500, 52.500, Math.toRadians(0));

    // Main shoot position
    private final Pose mainShootPose = new Pose(86.000, 79.000, Math.toRadians(45));

    // Gate paths
    private final Pose gatePose = new Pose(133.500, 57, Math.toRadians(30));
    private final Pose gateShootPose = new Pose(86.000, 79.000, Math.toRadians(30));

    // ShootToFourth path
    private final Pose fourthControlPoint = new Pose(107.500, 77.000);
    private final Pose fourthPickupPose = new Pose(127.000, 75.500, Math.toRadians(0));

    // ShootToFifth path
    private final Pose fifthControlPoint = new Pose(81.500, 24.500);
    private final Pose fifthPickupPose = new Pose(125.000, 29.000, Math.toRadians(0));

    // Stop
    private final Pose stopPose = new Pose(92.000, 109.500, Math.toRadians(45));

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

        ShootToGate1 = follower.pathBuilder()
                .addPath(new BezierLine(gateShootPose, gatePose))
                .setLinearHeadingInterpolation(gateShootPose.getHeading(), gatePose.getHeading())
                .setGlobalDeceleration()
                .build();

        GateToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, gateShootPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), gateShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        ShootToGate2 = follower.pathBuilder()
                .addPath(new BezierLine(gateShootPose, gatePose))
                .setLinearHeadingInterpolation(gateShootPose.getHeading(), gatePose.getHeading())
                .setGlobalDeceleration()
                .build();

        GateToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, gateShootPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), gateShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        ShootToFourth = follower.pathBuilder()
                .addPath(new BezierCurve(gateShootPose, fourthControlPoint, fourthPickupPose))
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
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Sorted;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .moveTo(startToShoot, maxSpeed, true)
                .moveTo(shootToFirst, maxSpeed, true)
                .moveTo(firstToShoot, maxSpeed, true)
                .moveTo(ShootToGate1, maxSpeed, true)
                .moveTo(GateToShoot1, maxSpeed, true)
                .moveTo(ShootToGate2, maxSpeed, true)
                .moveTo(GateToShoot2, maxSpeed, true)
                .moveTo(ShootToFourth, maxSpeed, true)
                .moveTo(fourthToShoot, maxSpeed, true)
                .moveTo(shootToFifth, maxSpeed, true)
                .moveTo(fifthToShootAndStop, maxSpeed, true)
                .moveTo(testingStop, maxSpeed, true)
                .build();
    }
}