package Autos.EighteenBalls;

import Autos.AutonTemplate;
import Constants.EnumConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;


@Autonomous(name = "Blue18BallV2")
public class Blue18BallV2 extends AutonTemplate {
    public static double maxSpeed = 1;

    private PathChain startToThirdSpike, thirdSpikeToShoot,
            shootToCorner, cornerToShoot,
            shootToSecond, shootToGate, gateToShoot,
            shootToFirst, firstToShoot,
            shootToCornerCycleOne, cornerToShootCycleOne,
            shootToEnd;

    // Start pose
    private final Pose startPose = new Pose(55.500, 6.750, Math.toRadians(90));

    // Shoot positions
    private final Pose shootPose = new Pose(54.500, 14.000, Math.toRadians(135));
    private final Pose shootPose150 = new Pose(54.500, 14.000, Math.toRadians(150));
    private final Pose upperShootPose = new Pose(48.000, 85.000, Math.toRadians(180));

    // Third spike
    private final Pose thirdSpikePrepPose = new Pose(38.000, 36.000, Math.toRadians(180));
    private final Pose thirdSpikePose = new Pose(17.500, 36.000, Math.toRadians(180));

    // Corner positions
    private final Pose cornerPrepPose = new Pose(24.500, 8.500, Math.toRadians(180));
    private final Pose cornerPose = new Pose(11.000, 8.500, Math.toRadians(180));
    private final Pose cornerCyclePose = new Pose(11.000, 9.500, Math.toRadians(180));

    // Second spike
    private final Pose secondPrepPose = new Pose(39.000, 60.000, Math.toRadians(180));
    private final Pose secondPose = new Pose(17.000, 60.000, Math.toRadians(180));

    // Gate
    private final Pose gatePose = new Pose(17.000, 64.500, Math.toRadians(180));

    // First spike & gate
    private final Pose firstSpikePose = new Pose(19.000, 85.000, Math.toRadians(180));

    // End pose
    private final Pose endPose = new Pose(49.000, 17.500, Math.toRadians(135));

    // Control points
    private final Pose startToThirdSpikeControl = new Pose(55.000, 33.600);
    private final Pose shootToSecondControl = new Pose(54.000, 50.500);
    private final Pose shootToGateControl = new Pose(23.000, 62.000);
    private final Pose gateToShootControl = new Pose(48.000, 66.500);

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        startToThirdSpike = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, startToThirdSpikeControl, thirdSpikePrepPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), thirdSpikePrepPose.getHeading())
                .addPath(new BezierLine(thirdSpikePrepPose, thirdSpikePose))
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
                .addPath(new BezierCurve(shootPose, shootToSecondControl, secondPrepPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondPrepPose.getHeading())
                .addPath(new BezierLine(secondPrepPose, secondPose))
                .setLinearHeadingInterpolation(secondPrepPose.getHeading(), secondPose.getHeading())
                .build();

        shootToGate = follower.pathBuilder()
                .addPath(new BezierCurve(secondPose, shootToGateControl, gatePose))
                .setLinearHeadingInterpolation(secondPose.getHeading(), gatePose.getHeading())
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
                .addPath(new BezierLine(firstSpikePose, shootPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        shootToCornerCycleOne = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, cornerPrepPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addPath(new BezierLine(cornerPrepPose, cornerCyclePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        cornerToShootCycleOne = follower.pathBuilder()
                .addPath(new BezierLine(cornerCyclePose, shootPose150))
                .setLinearHeadingInterpolation(cornerCyclePose.getHeading(), shootPose150.getHeading())
                .build();

        shootToEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose150, endPose))
                .setLinearHeadingInterpolation(shootPose150.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        super.init();
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Blue;
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
                .intakeStop()
                .moveTo(shootToGate, .8, false)
                .parallel(p -> p.delay(1).guaranteeSortedAutoCatalog())
                .moveTo(gateToShoot, maxSpeed, false)
                .slowShoot()
                .intakeStart()
                .moveTo(shootToFirst, maxSpeed, false)
                .parallel(p -> p.moveTo(firstToShoot, maxSpeed, false).guaranteeSortedAutoCatalog())
                .slowShoot()
                .intakeStart()
                .moveTo(shootToCornerCycleOne, maxSpeed, false)
                .delay(.3)
                .parallel(p -> p.moveTo(cornerToShootCycleOne, maxSpeed, false).guaranteeSortedAutoCatalog())
                .shoot()
                .moveTo(shootToEnd, maxSpeed, false)
                .build();
    }
}
