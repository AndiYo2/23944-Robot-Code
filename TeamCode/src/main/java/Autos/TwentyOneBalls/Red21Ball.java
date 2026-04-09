package Autos.TwentyOneBalls;

import Autos.AutonTemplate;
import Constants.EnumConstants;
import Constants.SpindexerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;


@Autonomous(name = "Red21Ball")
public class Red21Ball extends AutonTemplate {
    public static double maxSpeed = 1;
    private PathChain startToShoot, shootToSecondSpike, secondSpikeToShoot,
            shootToGateOne1, shootToGateOne2, gateOneToShoot,
            shootToGateTwo1, shootToGateTwo2, gateTwoToShoot,
            shootToFirstSpike, firstSpikeToShoot, shootToThirdSpike, thirdSpikeToShootEnd;

    // Start pose
    private final Pose startPose = new Pose(128.500, 109.500, Math.toRadians(90));

    // Shoot positions
    private final Pose shootPose = new Pose(88.500, 81.500, Math.toRadians(45));
    private final Pose shootPose2 = new Pose(88.500, 76.500, Math.toRadians(0));
    private final Pose shootPose2Angled = new Pose(88.500, 76.500, Math.toRadians(30));
    private final Pose endShootPose = new Pose(92.000, 108.000, Math.toRadians(45));

    // Spike positions
    private final Pose firstSpikePose = new Pose(125.500, 83.00, Math.toRadians(0));
    private final Pose secondSpikePose = new Pose(129.500, 57.750, Math.toRadians(0));
    private final Pose thirdSpikePose = new Pose(132.500, 33.500, Math.toRadians(0));

    // Gate positions
    private final Pose gateWaypointPose = new Pose(125.000, 61.500, Math.toRadians(31.5));
    private final Pose gatePose = new Pose(132.000, 59.000, Math.toRadians(31.5));

    // Control points
    private final Pose firstShootControl = new Pose(101.000, 101.500);
    private final Pose secondSpikeControl1 = new Pose(92.000, 56.000);
    private final Pose secondSpikeControl2 = new Pose(110.500, 57.500);
    private final Pose thirdSpikeControl1 = new Pose(87.500, 32.000);
    private final Pose thirdSpikeControl2 = new Pose(102.000, 33.500);

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        startToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, firstShootControl, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        shootToSecondSpike = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, secondSpikeControl1, secondSpikeControl2, secondSpikePose))
                .setLinearHeadingInterpolation(secondSpikePose.getHeading(), secondSpikePose.getHeading())
                .build();

        secondSpikeToShoot = follower.pathBuilder()
                .addPath(new BezierLine(secondSpikePose, shootPose2))
                .setLinearHeadingInterpolation(secondSpikePose.getHeading(), shootPose2.getHeading())
                .build();

        shootToGateOne1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, gateWaypointPose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), gateWaypointPose.getHeading())
                .build();

        shootToGateOne2 = follower.pathBuilder()
                .addPath(new BezierLine(gateWaypointPose, gatePose))
                .setLinearHeadingInterpolation(gateWaypointPose.getHeading(), gatePose.getHeading())
                .build();

        gateOneToShoot = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, shootPose2Angled))
                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose2Angled.getHeading())
                .build();

        shootToGateTwo1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2Angled, gateWaypointPose))
                .setLinearHeadingInterpolation(shootPose2Angled.getHeading(), gateWaypointPose.getHeading())
                .build();

        shootToGateTwo2 = follower.pathBuilder()
                .addPath(new BezierLine(gateWaypointPose, gatePose))
                .setLinearHeadingInterpolation(gateWaypointPose.getHeading(), gatePose.getHeading())
                .build();

        gateTwoToShoot = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, shootPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose2Angled.getHeading())
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
                .addPath(new BezierCurve(shootPose2, thirdSpikeControl1, thirdSpikeControl2, thirdSpikePose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), thirdSpikePose.getHeading())
                .build();

        thirdSpikeToShootEnd = follower.pathBuilder()
                .addPath(new BezierLine(thirdSpikePose, endShootPose))
                .setLinearHeadingInterpolation(thirdSpikePose.getHeading(), endShootPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        super.init();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .setShootWhileMoving(true)
                .parallel(p -> p.moveTo(startToShoot, maxSpeed, false).shootAfterDelay(.55))
                .setShootWhileMoving(false)
                .intakeStart()
                .moveTo(shootToSecondSpike, maxSpeed, false)
                .parallel(p -> p.moveTo(secondSpikeToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                .intakeStart()
                .moveTo(shootToGateOne1, maxSpeed, false)
                .moveTo(shootToGateOne2, maxSpeed, false)
                .delay(1)
                .parallel(p -> p.moveTo(gateOneToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                .intakeStart()
                .setSpindexerMode(EnumConstants.ShootingMode.Sorted)
                .moveTo(shootToGateTwo1, maxSpeed, false)
                .moveTo(shootToGateTwo2, maxSpeed, false)
                .delay(1)
                .parallel(p -> p.moveTo(gateTwoToShoot, maxSpeed, false).guaranteeSortedAutoCatalog())
                .slowShoot()
                .intakeStart()
                .moveTo(shootToFirstSpike, maxSpeed, false)
                .parallel(p -> p.moveTo(firstSpikeToShoot, maxSpeed, false).guaranteeSortedAutoCatalog())
                .slowShoot()
                .intakeStart()
                .moveTo(shootToThirdSpike, maxSpeed, false)
                .parallel(p -> p.moveTo(thirdSpikeToShootEnd, maxSpeed, false).guaranteeSortedAutoCatalog())
                .slowShoot()
                .setShootWhileMoving(true)
                .build();
    }
}
