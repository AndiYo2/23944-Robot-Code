package Autos.EighteenBalls;

import Autos.AutonTemplate;
import Constants.EnumConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;


@Autonomous(name = "Red18Ball")
public class Red18Ball extends AutonTemplate {
    public static double maxSpeed = 1;

    private PathChain startToThirdSpike, thirdSpikeToShoot,
            shootToCorner, cornerToShoot,
            shootToSecond, secondToGate, gateToShoot,
            shootToFirst, firstToShoot,
            shootToCornerCycleOne, cornerToShootCycleOne,
            shootToEnd;

    // Start pose
    private final Pose startPose = new Pose(88.500, 6.750, Math.toRadians(90));

    // Shoot positions
    private final Pose shootPose = new Pose(89.500, 14.000, Math.toRadians(45));
    private final Pose shootPose30 = new Pose(89.500, 14.000, Math.toRadians(30));
    private final Pose upperShootPose = new Pose(91.500, 80.000, Math.toRadians(0));
    private final Pose gateShootPose = new Pose(86.500, 74.000, Math.toRadians(0));

    // Third spike
    private final Pose thirdSpikePrepPose = new Pose(106.500, 21.500, Math.toRadians(45));
    private final Pose thirdSpikePose = new Pose(122.500, 27.500, Math.toRadians(45));

    // Corner positions
    private final Pose cornerPrepPose = new Pose(119.500, 8.500, Math.toRadians(0));
    private final Pose cornerPose = new Pose(133.000, 8.500, Math.toRadians(0));
    private final Pose cornerCyclePose = new Pose(133.000, 9.500, Math.toRadians(0));
    private final Pose cornerCycleMidPose = new Pose(101.000, 28.500, Math.toRadians(0));
    private final Pose cornerCycleControl = new Pose(104.500, 10.000);

    // Second spike
    private final Pose secondPrepPose = new Pose(106.500, 45.500, Math.toRadians(45));
    private final Pose secondPose = new Pose(124.500, 49.500, Math.toRadians(45));

    // Gate
    private final Pose gatePose = new Pose(127.000, 64.000, Math.toRadians(0));

    // First spike & gate
    private final Pose firstSpikePose = new Pose(126.500, 80.000, Math.toRadians(0));

    // End pose
    private final Pose endPose = new Pose(95.000, 17.500, Math.toRadians(45));

    // Control points
    private final Pose thirdSpikeControl = new Pose(112.000, 27.500);
    private final Pose secondControl = new Pose(112.000, 49.500);
    private final Pose secondToGateControl = new Pose(122.000, 58.000);
    private final Pose gateToShootControl = new Pose(96.000, 66.500);

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        startToThirdSpike = follower.pathBuilder()
                .addPath(new BezierLine(startPose, thirdSpikePrepPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), thirdSpikePrepPose.getHeading())
                .addPath(new BezierCurve(thirdSpikePrepPose, thirdSpikeControl, thirdSpikePose))
                .setLinearHeadingInterpolation(thirdSpikePrepPose.getHeading(), thirdSpikePose.getHeading())
                .build();

        thirdSpikeToShoot = follower.pathBuilder()
                .addPath(new BezierLine(thirdSpikePose, shootPose))
                .setLinearHeadingInterpolation(thirdSpikePose.getHeading(), shootPose.getHeading())
                .build();

        shootToCorner = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, cornerPrepPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), cornerPrepPose.getHeading())
                .addPath(new BezierLine(cornerPrepPose, cornerPose))
                .setLinearHeadingInterpolation(cornerPrepPose.getHeading(), cornerPose.getHeading())
                .build();

        cornerToShoot = follower.pathBuilder()
                .addPath(new BezierLine(cornerPose, shootPose))
                .setLinearHeadingInterpolation(cornerPose.getHeading(), shootPose.getHeading())
                .build();

        shootToSecond = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, secondPrepPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondPrepPose.getHeading())
                .addPath(new BezierCurve(secondPrepPose, secondControl, secondPose))
                .setLinearHeadingInterpolation(secondPrepPose.getHeading(), secondPose.getHeading())
                .build();

        secondToGate = follower.pathBuilder()
                .addPath(new BezierCurve(secondPose, secondToGateControl, gatePose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        gateToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gateToShootControl, upperShootPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), upperShootPose.getHeading())
                .build();

        shootToFirst = follower.pathBuilder()
                .addPath(new BezierLine(upperShootPose, firstSpikePose))
                .setLinearHeadingInterpolation(upperShootPose.getHeading(), firstSpikePose.getHeading())
                .build();


        firstToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstSpikePose, gateShootPose))
                .setLinearHeadingInterpolation(firstSpikePose.getHeading(), gateShootPose.getHeading())
                .build();

        shootToCornerCycleOne = follower.pathBuilder()
                .addPath(new BezierLine(gateShootPose, cornerCycleMidPose))
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(cornerCycleMidPose, cornerCycleControl, cornerCyclePose))
                .setTangentHeadingInterpolation()
                .build();

        cornerToShootCycleOne = follower.pathBuilder()
                .addPath(new BezierLine(cornerCyclePose, shootPose30))
                .setLinearHeadingInterpolation(cornerCyclePose.getHeading(), shootPose30.getHeading())
                .build();

        shootToEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose30, endPose))
                .setTangentHeadingInterpolation()
                .build();
    }

    @Override
    public void init() {
        super.init();
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;
        Constants.SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .parallel(p -> p.delay(.7).limelightScan())
                .shoot()
                .intakeStart()
                .moveTo(startToThirdSpike, maxSpeed, false)
                .parallel(p -> p.moveTo(thirdSpikeToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                .intakeStart()
                .moveTo(shootToCorner, maxSpeed, false)
                .delay(.4)
                .parallel(p -> p.moveTo(cornerToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                .setSpindexerMode(EnumConstants.ShootingMode.Sorted)
                .intakeStart()
                .moveTo(shootToSecond, maxSpeed, false)
                .moveTo(secondToGate, .8, false)
                .intakeStop()
                .delay(1)
                .parallel(p -> p.moveTo(gateToShoot, maxSpeed, false).guaranteeSortedAutoCatalog())
                .slowShoot()
                .intakeStart()
                .moveTo(shootToFirst, maxSpeed, false)
                .parallel(p -> p.moveTo(firstToShoot, maxSpeed, false).guaranteeSortedAutoCatalog())
                .slowShoot()
                .intakeStart()
                .moveTo(shootToCornerCycleOne, maxSpeed, false)
                .delay(.5)
                .parallel(p -> p.moveTo(cornerToShootCycleOne, maxSpeed, false).guaranteeSortedAutoCatalog())
                .slowShoot()
                .moveTo(shootToEnd, maxSpeed, false)
                .build();
    }
}