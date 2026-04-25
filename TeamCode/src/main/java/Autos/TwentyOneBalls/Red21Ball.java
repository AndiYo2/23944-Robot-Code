package Autos.TwentyOneBalls;

import Autos.AutonTemplate;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;


@Autonomous(name = "Red21Ball")
public class Red21Ball extends AutonTemplate {
    public static double maxSpeed = 1;

    private PathChain startToThirdSpike, thirdSpikeToShoot,
            shootToCorner, cornerToShoot,
            shootToSecond, secondToGate, gateToShoot,
            shootToFirst, firstSpikeToGate, firstSpikeGateToShoot,
            shootToCornerCycleOne, cornerToShootCycleOne,
            shootToCornerCycleTwo, cornerCycleTwoToShoot,
            shootToEnd;

    // Start pose
    private final Pose startPose = new Pose(88.500, 6.750, Math.toRadians(90));

    // Shoot positions
    private final Pose shootPose = new Pose(89.500, 14.000, Math.toRadians(45));
    private final Pose shootPose30 = new Pose(89.500, 14.000, Math.toRadians(30));
    private final Pose upperShootPose = new Pose(91.500, 83.000, Math.toRadians(0));
    private final Pose gateShootPose = new Pose(86.500, 74.000, Math.toRadians(0));

    // Third spike
    private final Pose thirdSpikePrepPose = new Pose(106.500, 21.500, Math.toRadians(45));
    private final Pose thirdSpikePose = new Pose(122.500, 27.500, Math.toRadians(45));

    // Corner positions
    private final Pose cornerPrepPose = new Pose(119.500, 8.500, Math.toRadians(0));
    private final Pose cornerPose = new Pose(131.000, 8.500, Math.toRadians(0));
    private final Pose cornerCycleOnePose = new Pose(130.000, 13.000, Math.toRadians(0));

    // Second spike
    private final Pose secondPrepPose = new Pose(106.500, 45.500, Math.toRadians(45));
    private final Pose secondPose = new Pose(122.500, 51.500, Math.toRadians(45));

    // Gate
    private final Pose gatePose = new Pose(126.500, 63.000, Math.toRadians(0));

    // First spike & gate
    private final Pose firstSpikeMidPose = new Pose(126.000, 83.000, Math.toRadians(0));
    private final Pose firstSpikeGatePose = new Pose(126.500, 72.500, Math.toRadians(0));

    // End pose
    private final Pose endPose = new Pose(95.000, 17.500, Math.toRadians(45));

    // Control points
    private final Pose thirdSpikeControl = new Pose(112.000, 27.500);
    private final Pose secondControl = new Pose(112.000, 51.500);
    private final Pose secondToGateControl = new Pose(122.000, 58.000);
    private final Pose gateToShootControl = new Pose(96.000, 66.500);
    private final Pose firstSpikeGateControl = new Pose(119.000, 78.000);

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
                .setLinearHeadingInterpolation(secondPose.getHeading(), gatePose.getHeading())
                .build();

        gateToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gateToShootControl, upperShootPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), upperShootPose.getHeading())
                .build();

        shootToFirst = follower.pathBuilder()
                .addPath(new BezierLine(upperShootPose, firstSpikeMidPose))
                .setLinearHeadingInterpolation(upperShootPose.getHeading(), firstSpikeMidPose.getHeading())
                .build();

        firstSpikeToGate = follower.pathBuilder()
                .addPath(new BezierCurve(firstSpikeMidPose, firstSpikeGateControl, firstSpikeGatePose))
                .setLinearHeadingInterpolation(firstSpikeMidPose.getHeading(), firstSpikeGatePose.getHeading())
                .build();

        firstSpikeGateToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstSpikeGatePose, gateShootPose))
                .setLinearHeadingInterpolation(firstSpikeGatePose.getHeading(), gateShootPose.getHeading())
                .build();

        shootToCornerCycleOne = follower.pathBuilder()
                .addPath(new BezierLine(gateShootPose, cornerCycleOnePose))
                .setTangentHeadingInterpolation()
                .build();

        cornerToShootCycleOne = follower.pathBuilder()
                .addPath(new BezierLine(cornerCycleOnePose, shootPose30))
                .setLinearHeadingInterpolation(cornerCycleOnePose.getHeading(), shootPose30.getHeading())
                .build();

        shootToCornerCycleTwo = follower.pathBuilder()
                .addPath(new BezierLine(shootPose30, cornerPrepPose))
                .setLinearHeadingInterpolation(shootPose30.getHeading(), cornerPrepPose.getHeading())
                .addPath(new BezierLine(cornerPrepPose, cornerPose))
                .setLinearHeadingInterpolation(cornerPrepPose.getHeading(), cornerPose.getHeading())
                .build();

        cornerCycleTwoToShoot = follower.pathBuilder()
                .addPath(new BezierLine(cornerPose, shootPose30))
                .setLinearHeadingInterpolation(cornerPose.getHeading(), shootPose30.getHeading())
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

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .parallel(p -> p.delay(.5).limelightScan())
                .shoot()
                .intakeStart()
                .moveTo(startToThirdSpike, maxSpeed, false)
                .delay(.3)
                .parallel(p -> p.moveTo(thirdSpikeToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                .intakeStart()
                .moveTo(shootToCorner, maxSpeed, false)
                .parallel(p -> p.moveTo(cornerToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                .intakeStart()
                .moveTo(shootToSecond, maxSpeed, false)
                .moveTo(secondToGate, maxSpeed, false)
                .autoCatalog()
                .moveTo(gateToShoot, maxSpeed, false)
                .shoot()
                .intakeStart()
                .moveTo(shootToFirst, maxSpeed, false)
                .moveTo(firstSpikeToGate, maxSpeed, false)
                .delay(.3)
                .parallel(p -> p.moveTo(firstSpikeGateToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                .intakeStart()
                .moveTo(shootToCornerCycleOne, maxSpeed, false)
                .moveTo(cornerToShootCycleOne, maxSpeed, false)
                .shoot()
                .intakeStart()
                .moveTo(shootToCornerCycleTwo, maxSpeed, false)
                .parallel(p -> p.moveTo(cornerCycleTwoToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                .moveTo(shootToEnd, maxSpeed, false)
                .build();
    }
}