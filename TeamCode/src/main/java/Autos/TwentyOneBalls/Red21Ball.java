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
    public static double maxSpeed = .85;
    private PathChain startToShoot, shootToSpikeOne, spikeOneToGateEntry,
            gateEntryToGateOpen, gateOpenToFirstPickup, firstPickupToSecondPickup,
            secondPickupToThirdPickup, thirdPickupToFifthSpike, fifthSpikeToShoot;

    // Named pose constants
    private final Pose startPose = new Pose(128.000, 112.000, Math.toRadians(0));
    private final Pose initialShootPose = new Pose(115.000, 100.000, Math.toRadians(15));
    private final Pose spikeOnePose = new Pose(119.500, 90.500, Math.toRadians(270));
    private final Pose spikeTwoPose = new Pose(108.500, 91.000, Math.toRadians(270));
    private final Pose spikeThreePose = new Pose(102.000, 98.500, Math.toRadians(30));
    private final Pose spikeShootPose = new Pose(93.000, 82.000, Math.toRadians(30));
    private final Pose gateApproachPose = new Pose(110.500, 59.000, Math.toRadians(0));
    private final Pose gateEntryPose = new Pose(126.500, 59.000, Math.toRadians(0));
    private final Pose gateOpenPose = new Pose(127.000, 66.500, Math.toRadians(0));
    private final Pose mainShootPose = new Pose(97.000, 112.000, Math.toRadians(30));
    private final Pose gatePickupPose = new Pose(132.500, 57.500, Math.toRadians(45));
    private final Pose fifthSpikePrepPose = new Pose(105.500, 34.000, Math.toRadians(0));
    private final Pose fifthSpikePose = new Pose(126.000, 34.000, Math.toRadians(0));
    private final Pose lastShootPose = new Pose(91.500, 128.000, Math.toRadians(45));

    // Control points
    private final Pose shootToSpikeControlPoint = new Pose(120.000, 110.000);
    private final Pose spikeShootControlPoint = new Pose(93.500, 110.500);
    private final Pose gateApproachControlPoint = new Pose(91.798, 62.117);
    private final Pose gateReturnControlPoint = new Pose(112.500, 126.000);
    private final Pose gatePickupControlPoint1 = new Pose(129.000, 90.300);
    private final Pose gatePickupControlPoint2 = new Pose(119.000, 61.500);
    private final Pose gateLoopControlPoint1 = new Pose(114.500, 63.000);
    private final Pose gateLoopControlPoint2 = new Pose(127.500, 102.500);
    private final Pose gateLoopControlPoint3 = new Pose(109.500, 121.000);
    private final Pose firstReturnControlPoint = new Pose(95.809, 77.755);
    private final Pose secondReturnControlPoint = new Pose(96.000, 77.755);
    private final Pose lastReturnControlPoint1 = new Pose(103.500, 69.500);
    private final Pose lastReturnControlPoint2 = new Pose(116.500, 115.000);
    private final Pose fifthSpikeControlPoint1 = new Pose(74.000, 92.000);
    private final Pose fifthSpikeControlPoint2 = new Pose(90.500, 34.500);

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        // Path 1
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, initialShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), initialShootPose.getHeading())
                .build();

        // Path 2
        shootToSpikeOne = follower.pathBuilder()
                .addPath(new BezierCurve(initialShootPose, shootToSpikeControlPoint, spikeOnePose))
                .setLinearHeadingInterpolation(Math.toRadians(0), spikeOnePose.getHeading())
                .build();

        // Paths 3-7 (continuous)
        spikeOneToGateEntry = follower.pathBuilder()
                .addPath(new BezierLine(spikeOnePose, spikeTwoPose))
                .setConstantHeadingInterpolation(spikeOnePose.getHeading())
                .addPath(new BezierLine(spikeTwoPose, spikeThreePose))
                .setLinearHeadingInterpolation(spikeTwoPose.getHeading(), spikeThreePose.getHeading())
                .addPath(new BezierCurve(spikeThreePose, spikeShootControlPoint, spikeShootPose))
                .setLinearHeadingInterpolation(spikeThreePose.getHeading(), spikeShootPose.getHeading())
                .addPath(new BezierCurve(spikeShootPose, gateApproachControlPoint, gateApproachPose))
                .setLinearHeadingInterpolation(spikeShootPose.getHeading(), gateApproachPose.getHeading())
                .addPath(new BezierLine(gateApproachPose, gateEntryPose))
                .setLinearHeadingInterpolation(gateApproachPose.getHeading(), gateEntryPose.getHeading())
                .build();

        // Path 8
        gateEntryToGateOpen = follower.pathBuilder()
                .addPath(new BezierLine(gateEntryPose, gateOpenPose))
                .setConstantHeadingInterpolation(gateEntryPose.getHeading())
                .build();

        // Paths 9-10 (continuous)
        gateOpenToFirstPickup = follower.pathBuilder()
                .addPath(new BezierCurve(gateOpenPose, gateReturnControlPoint, mainShootPose))
                .setLinearHeadingInterpolation(gateOpenPose.getHeading(), mainShootPose.getHeading())
                .addPath(new BezierCurve(mainShootPose, gatePickupControlPoint1, gatePickupControlPoint2, gatePickupPose))
                .setLinearHeadingInterpolation(mainShootPose.getHeading(), gatePickupPose.getHeading())
                .build();

        // Paths 11-12 (continuous)
        firstPickupToSecondPickup = follower.pathBuilder()
                .addPath(new BezierCurve(gatePickupPose, gateLoopControlPoint1, gateLoopControlPoint2, gateLoopControlPoint3, mainShootPose))
                .setLinearHeadingInterpolation(gatePickupPose.getHeading(), mainShootPose.getHeading())
                .addPath(new BezierCurve(mainShootPose, firstReturnControlPoint, gatePickupPose))
                .setLinearHeadingInterpolation(mainShootPose.getHeading(), gatePickupPose.getHeading())
                .build();

        // Paths 13-14 (continuous)
        secondPickupToThirdPickup = follower.pathBuilder()
                .addPath(new BezierCurve(gatePickupPose, gateLoopControlPoint1, gateLoopControlPoint2, gateLoopControlPoint3, mainShootPose))
                .setLinearHeadingInterpolation(gatePickupPose.getHeading(), mainShootPose.getHeading())
                .addPath(new BezierCurve(mainShootPose, secondReturnControlPoint, gatePickupPose))
                .setLinearHeadingInterpolation(mainShootPose.getHeading(), gatePickupPose.getHeading())
                .build();

        // Paths 15-17 (continuous)
        thirdPickupToFifthSpike = follower.pathBuilder()
                .addPath(new BezierCurve(gatePickupPose, lastReturnControlPoint1, lastReturnControlPoint2, mainShootPose))
                .setLinearHeadingInterpolation(gatePickupPose.getHeading(), mainShootPose.getHeading())
                .addPath(new BezierCurve(mainShootPose, fifthSpikeControlPoint1, fifthSpikeControlPoint2, fifthSpikePrepPose))
                .setLinearHeadingInterpolation(mainShootPose.getHeading(), fifthSpikePrepPose.getHeading())
                .addPath(new BezierLine(fifthSpikePrepPose, fifthSpikePose))
                .setLinearHeadingInterpolation(fifthSpikePrepPose.getHeading(), fifthSpikePose.getHeading())
                .build();

        // Path 18
        fifthSpikeToShoot = follower.pathBuilder()
                .addPath(new BezierLine(fifthSpikePose, lastShootPose))
                .setLinearHeadingInterpolation(fifthSpikePose.getHeading(), lastShootPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        super.init();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .moveTo(startToShoot, maxSpeed, false)
                .moveTo(shootToSpikeOne, maxSpeed, false)
                .moveTo(spikeOneToGateEntry, maxSpeed, false)
                .moveTo(gateEntryToGateOpen, maxSpeed, false)
                .moveTo(gateOpenToFirstPickup, maxSpeed, false)
                .moveTo(firstPickupToSecondPickup, maxSpeed, false)
                .moveTo(secondPickupToThirdPickup, maxSpeed, false)
                .moveTo(thirdPickupToFifthSpike, maxSpeed, false)
                .moveTo(fifthSpikeToShoot, maxSpeed, false)
                .build();
    }
}