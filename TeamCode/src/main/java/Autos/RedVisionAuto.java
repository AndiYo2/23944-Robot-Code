package Autos;

import Constants.EnumConstants;
import Constants.SpindexerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;
import vision.ArtifactDetector;


@Autonomous(name = "RedVisionAuto")
public class RedVisionAuto extends AutonTemplate {
    public static double maxSpeed = 1;
    private PathChain shootToFirst, firstToShoot, shootToSecond, secondToShoot, shootToScan;
    private PathChain[] visionGoToPaths;
    private ArtifactDetector detector;

    // Start pose
    private final Pose startPose = new Pose(88.500, 6.750, Math.toRadians(90));

    // Start curve control point
    private final Pose startCurveControl = new Pose(108.500, 14.000);

    // Cycle 1 pickup (corner balls)
    private final Pose cornerBallsPickupPose = new Pose(132.000, 8.500, Math.toRadians(0));

    // Shoot position
    private final Pose shootPose = new Pose(88.500, 14.500, Math.toRadians(30));

    // Scan position (slightly forward, heading 0 deg for camera FOV)
    private final Pose scanPose = new Pose(90.500, 14.500, Math.toRadians(0));

    // Cycle 2 (third spike — fluid two-segment path)
    private final Pose thirdSpikeCurveControl = new Pose(91.500, 29.500);
    private final Pose thirdSpikeMidPose = new Pose(102.000, 35.000, Math.toRadians(0));
    private final Pose thirdSpikePickupPose = new Pose(131.000, 35.000, Math.toRadians(0));

    // Vision lane 1 (fluid: line + line)
    private final Pose lane1MidPose = new Pose(102.000, 8.500, Math.toRadians(0));
    private final Pose lane1Pickup = new Pose(132.000, 8.500, Math.toRadians(0));

    // Vision lane 2 (single curve)
    private final Pose lane2Control = new Pose(105.500, 26.000);
    private final Pose lane2Pickup = new Pose(132.000, 24.500, Math.toRadians(0));

    // Vision lane 3 (fluid: curve + line)
    private final Pose lane3CurveControl = new Pose(91.500, 32.500);
    private final Pose lane3MidPose = new Pose(106.000, 40.500, Math.toRadians(0));
    private final Pose lane3Pickup = new Pose(132.000, 40.500, Math.toRadians(0));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        // Cycle 1 outbound (corner balls)
        shootToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, startCurveControl, cornerBallsPickupPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), cornerBallsPickupPose.getHeading())
                .build();

        // Cycle 1 return
        firstToShoot = follower.pathBuilder()
                .addPath(new BezierLine(cornerBallsPickupPose, shootPose))
                .setLinearHeadingInterpolation(cornerBallsPickupPose.getHeading(), shootPose.getHeading())
                .build();

        // Cycle 2 outbound (third spike — fluid: curve + line)
        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, thirdSpikeCurveControl, thirdSpikeMidPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), thirdSpikeMidPose.getHeading())
                .addPath(new BezierLine(thirdSpikeMidPose, thirdSpikePickupPose))
                .setLinearHeadingInterpolation(thirdSpikeMidPose.getHeading(), thirdSpikePickupPose.getHeading())
                .build();

        // Cycle 2 return
        secondToShoot = follower.pathBuilder()
                .addPath(new BezierLine(thirdSpikePickupPose, shootPose))
                .setLinearHeadingInterpolation(thirdSpikePickupPose.getHeading(), shootPose.getHeading())
                .build();

        // Rotate to scan heading (short nudge forward while turning 30 deg -> 0 deg)
        shootToScan = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, scanPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), scanPose.getHeading())
                .build();

        // Vision collection paths (outbound only — return is dynamic)

        // Lane 1 (fluid: line + line)
        PathChain visionPath1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, lane1MidPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(lane1MidPose, lane1Pickup))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Lane 2 (single curve)
        PathChain visionPath2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, lane2Control, lane2Pickup))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Lane 3 (fluid: curve + line)
        PathChain visionPath3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, lane3CurveControl, lane3MidPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(lane3MidPose, lane3Pickup))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Indexed by ChosenPath ordinal: PATH_1=0, PATH_2=1, PATH_3=2
        visionGoToPaths = new PathChain[]{ visionPath1, visionPath2, visionPath3 };
    }

    @Override
    public void init() {
        super.init();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;

        detector = new ArtifactDetector();

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .delay(.35)
                .shoot()
                // Cycle 1 (corner balls)
                .intakeStart()
                .moveTo(shootToFirst, maxSpeed, false)
                .delay(.75)
                .parallel(p -> p.moveTo(firstToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                // Cycle 2 (third spike)
                .intakeStart()
                .moveTo(shootToSecond, maxSpeed, false)
                .delay(.15)
                .parallel(p -> p.moveTo(secondToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                // Rotate to scan heading
                .moveTo(shootToScan, maxSpeed, false)
                // Cycle 3 (vision)
                .intakeStart()
                .visionCollectAndCatalog(detector, visionGoToPaths, shootPose, maxSpeed, false, .3)
                .shoot()
                // Rotate to scan heading
                .moveTo(shootToScan, maxSpeed, false)
                // Cycle 4 (vision)
                .intakeStart()
                .visionCollectAndCatalog(detector, visionGoToPaths, shootPose, maxSpeed, false, .3)
                .shoot()
                // Rotate to scan heading
                .moveTo(shootToScan, maxSpeed, false)
                // Cycle 5 (vision)
                .intakeStart()
                .visionCollectAndCatalog(detector, visionGoToPaths, shootPose, maxSpeed, false, .3)
                .shoot()
                // Rotate to scan heading
                .moveTo(shootToScan, maxSpeed, false)
                // Cycle 6 (vision)
                .intakeStart()
                .visionCollectAndCatalog(detector, visionGoToPaths, shootPose, maxSpeed, false, .3)
                .shoot()
                // Rotate to scan heading
                .moveTo(shootToScan, maxSpeed, false)
                // Cycle 7 (vision)
                .intakeStart()
                .visionCollectAndCatalog(detector, visionGoToPaths, shootPose, maxSpeed, false, .3)
                .shoot()
                .build();
    }

    @Override
    public void stop() {
        super.stop();
        if (detector != null) {
            detector.disable();
        }
    }
}
