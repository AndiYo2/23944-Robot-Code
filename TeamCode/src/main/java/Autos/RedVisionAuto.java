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
    private PathChain shootToFirst, firstToShoot, shootToSecond, secondToShoot, shootToScan, shootToStop;
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

    // Vision lane paths are gone — CorridorSelector builds the path at runtime
    // from the live ball positions.

    private final Pose stopPose = new Pose(94.500, 20.500, Math.toRadians(30));

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

        shootToStop = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, stopPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), stopPose.getHeading())
                .build();

        // Vision collection path is now built at runtime inside VisionCollectCommand
        // based on the detected corridor. No pre-built lane paths needed.
    }

    @Override
    public void init() {
        super.init();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;

        detector = new ArtifactDetector();

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .delay(.6)
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
                // Cycle 3 (vision — dual scan: 30 deg then 0 deg)
                .visionPreScan(detector)
                .moveTo(shootToScan, maxSpeed, false)
                .intakeStart()
                .visionCollectAndCatalog(detector, shootPose, maxSpeed, false, .3)
                .shoot()
                // Cycle 4 (vision)
                .visionPreScan(detector)
                .moveTo(shootToScan, maxSpeed, false)
                .intakeStart()
                .visionCollectAndCatalog(detector, shootPose, maxSpeed, false, .3)
                .shoot()
                // Cycle 5 (vision)
                .visionPreScan(detector)
                .moveTo(shootToScan, maxSpeed, false)
                .intakeStart()
                .visionCollectAndCatalog(detector, shootPose, maxSpeed, false, .3)
                .shoot()
                .moveTo(shootToStop)
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
