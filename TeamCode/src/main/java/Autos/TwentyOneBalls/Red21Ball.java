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
            shootToGateOne, gateOneToShoot, shootToGateTwo, gateTwoToShoot,
            shootToFirstSpike, firstSpikeToShoot, shootToThirdSpike, thirdSpikeToShootEnd;

    // Start pose
    private final Pose startPose = new Pose(128.500, 109.500, Math.toRadians(90));

    // Shoot positions
    private final Pose shootPose = new Pose(88.500, 81.500, Math.toRadians(45));
    private final Pose shootPose2 = new Pose(88.500, 76.500, Math.toRadians(0));
    private final Pose shootPose2Angled = new Pose(88.500, 76.500, Math.toRadians(30));
    private final Pose endShootPose = new Pose(92.000, 108.000, Math.toRadians(45));

    // Spike positions
    private final Pose firstSpikePose = new Pose(122.500, 81.500, Math.toRadians(0));
    private final Pose secondSpikePose = new Pose(122.500, 57.750, Math.toRadians(0));
    private final Pose thirdSpikePose = new Pose(122.500, 33.500, Math.toRadians(0));

    // Gate position
    private final Pose gatePose = new Pose(132.000, 59.000, Math.toRadians(45));

    // Control points
    private final Pose secondSpikeControl1 = new Pose(92.000, 56.000);
    private final Pose secondSpikeControl2 = new Pose(110.500, 57.500);
    private final Pose thirdSpikeControl1 = new Pose(87.500, 32.000);
    private final Pose thirdSpikeControl2 = new Pose(102.000, 33.500);

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

        shootToSecondSpike = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, secondSpikeControl1, secondSpikeControl2, secondSpikePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        secondSpikeToShoot = follower.pathBuilder()
                .addPath(new BezierLine(secondSpikePose, shootPose2))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        shootToGateOne = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, gatePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        gateOneToShoot = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, shootPose2Angled))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(30))
                .build();

        shootToGateTwo = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2Angled, gatePose))
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(45))
                .build();

        gateTwoToShoot = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(30))
                .build();

        shootToFirstSpike = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, firstSpikePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        firstSpikeToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstSpikePose, shootPose2))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        shootToThirdSpike = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, thirdSpikeControl1, thirdSpikeControl2, thirdSpikePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        thirdSpikeToShootEnd = follower.pathBuilder()
                .addPath(new BezierLine(thirdSpikePose, endShootPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
    }

    @Override
    public void init() {
        super.init();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .delay(.5)
                .setShootWhileMoving(true)
                .parallel(p -> p.moveTo(startToShoot, maxSpeed, false).shoot())
                .setShootWhileMoving(false)
                .intakeStart()
                .moveTo(shootToSecondSpike, maxSpeed, false)
                .parallel(p -> p.moveTo(secondSpikeToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                .intakeStart()
                .moveTo(shootToGateOne, maxSpeed, false)
                .parallel(p -> p.moveTo(gateOneToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                .intakeStart()
                .setSpindexerMode(EnumConstants.ShootingMode.Sorted)
                .moveTo(shootToGateTwo, maxSpeed, false)
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
                .build();
    }
}
