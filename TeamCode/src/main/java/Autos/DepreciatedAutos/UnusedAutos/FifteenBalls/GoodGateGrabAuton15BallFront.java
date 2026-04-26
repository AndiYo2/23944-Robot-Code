package Autos.DepreciatedAutos.UnusedAutos.FifteenBalls;

import Autos.AutonTemplate;
import Constants.EnumConstants;
import Constants.SpindexerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import commands.CommandSequenceBuilder;

@Disabled
@Autonomous(name = "15TestSorted")
public class GoodGateGrabAuton15BallFront extends AutonTemplate {
    public static double maxSpeed = 1;

    private PathChain startToShoot, shootToSecondSpike, secondSpikeToShoot,
            shootToGateOne, gateTwoGrab, gateGrabToShoot,
            shootToFirstSpike, firstSpikeToShoot,
            shootToThirdSpike, thirdSpikeToShootEnd;

    // Start pose
    private final Pose startPose = new Pose(128.500, 109.500, Math.toRadians(90));

    // Shoot positions
    private final Pose shootPose = new Pose(88.500, 81.500, Math.toRadians(0));
    private final Pose shootPose2 = new Pose(88.500, 76.500, Math.toRadians(30));

    // Spike positions
    private final Pose firstSpikePose = new Pose(125.500, 83.000, Math.toRadians(0));
    private final Pose secondSpikePose = new Pose(129.500, 57.000, Math.toRadians(0));
    private final Pose thirdSpikeMidPose = new Pose(102.000, 34.500, Math.toRadians(0));
    private final Pose thirdSpikePose = new Pose(132.500, 34.500, Math.toRadians(0));

    // Gate positions
    private final Pose gateOnePose = new Pose(129.500, 60.000, Math.toRadians(20));
    private final Pose gateGrabPose = new Pose(132.000, 50.000, Math.toRadians(20));
    private final Pose gateGrabMidPose = new Pose(113.500, 53.000, Math.toRadians(20));

    // End pose
    private final Pose endPose = new Pose(92.000, 108.000, Math.toRadians(45));

    // Control points
    private final Pose startShootControl = new Pose(101.000, 101.500);
    private final Pose secondSpikeControl1 = new Pose(92.000, 55.000);
    private final Pose secondSpikeControl2 = new Pose(110.500, 57.500);
    private final Pose gateToShootControl = new Pose(89.000, 58.000);
    private final Pose thirdSpikeControl = new Pose(88.500, 40.000);

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        startToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, startShootControl, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), startPose.getHeading())
                .build();

        shootToSecondSpike = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, secondSpikeControl1, secondSpikeControl2, secondSpikePose))
                .setLinearHeadingInterpolation(secondSpikePose.getHeading(), secondSpikePose.getHeading())
                .build();

        secondSpikeToShoot = follower.pathBuilder()
                .addPath(new BezierLine(secondSpikePose, shootPose2))
                .setLinearHeadingInterpolation(secondSpikePose.getHeading(), shootPose2.getHeading())
                .build();

        shootToGateOne = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, gateOnePose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), gateOnePose.getHeading())
                .build();

        gateTwoGrab = follower.pathBuilder()
                .addPath(new BezierLine(gateOnePose, gateGrabPose))
                .setLinearHeadingInterpolation(gateOnePose.getHeading(), gateGrabPose.getHeading())
                .build();

        gateGrabToShoot = follower.pathBuilder()
                .addPath(new BezierLine(gateGrabPose, gateGrabMidPose))
                .setLinearHeadingInterpolation(gateGrabPose.getHeading(), gateGrabMidPose.getHeading())
                .addPath(new BezierCurve(gateGrabMidPose, gateToShootControl, shootPose))
                .setLinearHeadingInterpolation(gateGrabMidPose.getHeading(), Math.toRadians(30))
                .build();

        shootToFirstSpike = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, firstSpikePose))
                .setLinearHeadingInterpolation(firstSpikePose.getHeading(), firstSpikePose.getHeading())
                .build();

        firstSpikeToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstSpikePose, shootPose2))
                .setLinearHeadingInterpolation(firstSpikePose.getHeading(), shootPose2.getHeading())
                .build();

        shootToThirdSpike = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, thirdSpikeControl, thirdSpikeMidPose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), thirdSpikeMidPose.getHeading())
                .addPath(new BezierLine(thirdSpikeMidPose, thirdSpikePose))
                .setTangentHeadingInterpolation()
                .build();

        thirdSpikeToShootEnd = follower.pathBuilder()
                .addPath(new BezierLine(thirdSpikePose, endPose))
                .setLinearHeadingInterpolation(thirdSpikePose.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        super.init();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .setShootWhileMoving(false)
                .rampClear()
                .moveTo(startToShoot, maxSpeed, false)
                .delay(0.2)
                .parallel(p -> p.shoot().limelightScan())
                .intakeStart()
                .moveTo(shootToSecondSpike, maxSpeed, false)
                .parallel(p -> p.moveTo(secondSpikeToShoot, maxSpeed, false).autoCatalog())
                .delay(0.2)
                .shoot()
                .setSpindexerMode(EnumConstants.ShootingMode.Sorted)
                .intakeStart()
                .moveToParametric(shootToGateOne, t -> t < 0.8 ? maxSpeed : 0.8, false)
                .delay(0.5)
                .moveTo(gateTwoGrab, maxSpeed, false)
                .delay(0.5)
                .parallel(p -> p.moveTo(gateGrabToShoot, maxSpeed, false).guaranteeSortedAutoCatalog())
                .delay(0.2)
                .slowShoot()
                .intakeStart()
                .moveTo(shootToFirstSpike, maxSpeed, false)
                .parallel(p -> p.moveTo(firstSpikeToShoot, maxSpeed, false).guaranteeSortedAutoCatalog())
                .delay(0.2)
                .slowShoot()
                .intakeStart()
                .moveTo(shootToThirdSpike, maxSpeed, false)
                .parallel(p -> p.moveTo(thirdSpikeToShootEnd, maxSpeed, false).guaranteeSortedAutoCatalog())
                .delay(0.2)
                .slowShoot()
                .build();
    }
}
