package Autos.PartnerAutos;

import Autos.AutonTemplate;
import Constants.EnumConstants;
import Constants.SpindexerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;
import vision.ArtifactDetector;


@Autonomous(name = "BlueVisionAuto")
public class BlueVisionAuto extends AutonTemplate {
    public static double maxSpeed = 1;
    private PathChain shootToFirst, firstToShoot, shootToSecond, secondToShoot, shootToScan, shootToStop;
    private PathChain scanToCornerFallback;
    private ArtifactDetector detector;

    // Mirror of RedVisionAuto with X_blue = 144 - X_red, Y unchanged,
    // heading_blue = 180 - heading_red (i.e. π - h).

    // Start pose
    private final Pose startPose = new Pose(55.500, 6.750, Math.toRadians(90));

    // Start curve control point
    private final Pose startCurveControl = new Pose(35.500, 14.000);

    // Cycle 1 pickup (corner balls)
    private final Pose cornerBallsPickupPose = new Pose(12.000, 8.500, Math.toRadians(180));

    // Shoot position
    private final Pose shootPose = new Pose(55.500, 14.500, Math.toRadians(150));

    // Scan position. Robot stops here facing 180° to scan the field. If no balls
    // are found at 180°, the auto rotates in place to 150° and re-scans.
    private final Pose scanPose = new Pose(55.500, 10.000, Math.toRadians(180));

    // Cycle 2 (third spike — fluid two-segment path)
    private final Pose thirdSpikeCurveControl = new Pose(52.500, 29.500);
    private final Pose thirdSpikeMidPose = new Pose(42.000, 35.000, Math.toRadians(180));
    private final Pose thirdSpikePickupPose = new Pose(13.000, 35.000, Math.toRadians(180));

    private final Pose stopPose = new Pose(49.500, 20.500, Math.toRadians(150));

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

        // Rotate to scan heading (short nudge forward while turning 150° -> 180°)
        shootToScan = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, scanPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), scanPose.getHeading())
                .build();

        shootToStop = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, stopPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), stopPose.getHeading())
                .build();

        // Fallback drive for vision cycles: if BOTH the 180° scan and the
        // rotated 150° re-scan are empty, drive from scanPose to the corner
        // ball pickup pose (same destination as cycle 1). Starts at scanPose,
        // not shootPose, so the geometry is different from shootToFirst.
        scanToCornerFallback = follower.pathBuilder()
                .addPath(new BezierLine(scanPose, cornerBallsPickupPose))
                .setLinearHeadingInterpolation(scanPose.getHeading(), cornerBallsPickupPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        super.init();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Blue;

        detector = new ArtifactDetector();

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .delay(.6)
                .shoot()
                // Cycle 1 (corner balls)
                .intakeStart()
                .moveTo(shootToFirst, maxSpeed, false)
                .delay(.4)
                .parallel(p -> p.moveTo(firstToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                // Cycle 2 (third spike)
                .intakeStart()
                .moveTo(shootToSecond, maxSpeed, false)
                .delay(.15)
                .parallel(p -> p.moveTo(secondToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                // Cycle 3 (vision — dual scan: 150° then 180°)
                .moveTo(shootToScan, maxSpeed, false)
                .intakeStart()
                .visionCollectWithFallbacksAndCatalog(detector, Math.toRadians(150), scanToCornerFallback, shootPose, maxSpeed, false, .3)
                .shoot()
                // Cycle 4 (vision)
                .moveTo(shootToScan, maxSpeed, false)
                .intakeStart()
                .visionCollectWithFallbacksAndCatalog(detector, Math.toRadians(150), scanToCornerFallback, shootPose, maxSpeed, false, .3)
                .shoot()
                // Cycle 5 (vision)
                .moveTo(shootToScan, maxSpeed, false)
                .intakeStart()
                .visionCollectWithFallbacksAndCatalog(detector, Math.toRadians(150), scanToCornerFallback, shootPose, maxSpeed, false, .3)
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
