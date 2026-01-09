package Autos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import framework.AutonSequence;

@Autonomous(name = "BlueBackAutonCorner (New Framework)", group = "Autonomous")
public class BlueBackAutonCorner extends AutonTemplate {
    private Path shootToFirst;
    private PathChain firstToShoot, shootToSecondOne, shootToSecondTwo, shootToSecondThree, secondToShoot, secondShootToStop;

    private final Pose startPose = new Pose(56.5, 8.5, Math.toRadians(90));
    private final Pose endPose = new Pose(54.5, 40, Math.toRadians(90));

    private final Pose firstPickupPose = new Pose(23, 37.5, Math.toRadians(180));
    private final Pose firstPickupPoseControlPoint = new Pose(56, 31);

    private final Pose firstShootPose = new Pose(56.5, 15.5, 135);


    private final Pose secondPickupPose1 = new Pose(13, 49.5, 255);
    private final Pose secondPickupPose2 = new Pose(11.25, 22.5, 255);
    private final Pose secondPickupPose3 = new Pose(11, 9.5, 270);


    private final Pose secondShootPose = new Pose(54.5, 13, 90);
    private final Pose secondShootPoseControlPoint = new Pose(29.5, 20);





    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        shootToFirst = new Path(new BezierCurve(startPose, firstPickupPoseControlPoint, firstPickupPose));
        shootToFirst.setLinearHeadingInterpolation(startPose.getHeading(), firstPickupPose.getHeading());

        firstToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstPickupPose, firstShootPose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), firstShootPose.getHeading())
                .build();

        shootToSecondOne = follower.pathBuilder()
                .addPath(new BezierLine(firstShootPose, secondPickupPose1))
                .setLinearHeadingInterpolation(firstShootPose.getHeading(), secondPickupPose1.getHeading())
                .build();

        shootToSecondTwo = follower.pathBuilder()
                .addPath(new BezierLine(secondPickupPose1, secondPickupPose2))
                .setLinearHeadingInterpolation(secondPickupPose1.getHeading(), secondPickupPose2.getHeading())
                .build();

        shootToSecondThree = follower.pathBuilder()
                .addPath(new BezierLine(secondPickupPose2, secondPickupPose3))
                .setLinearHeadingInterpolation(secondPickupPose2.getHeading(), secondPickupPose3.getHeading())
                .build();

        secondToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(secondPickupPose3, secondShootPoseControlPoint, secondShootPose))
                .setLinearHeadingInterpolation(secondPickupPose3.getHeading(), secondShootPose.getHeading())
                .build();

        secondShootToStop = follower.pathBuilder()
                .addPath(new BezierLine(secondShootPose, endPose))
                .setLinearHeadingInterpolation(secondShootPose.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        // Not used - framework handles execution
    }

    @Override
    public void init() {
        super.init();

        // Build the autonomous sequence using the fluent API
        executor = new AutonSequence(follower, intake, catalogManager, sequenceManager, limelight)
                // Initial: scan and set preloaded balls (PPG), then shoot
                .parallel(p -> p.limelightScan().preload())
                .shoot()

                // Move to first pickup with intake running
                .parallel(p -> p.moveTo(shootToFirst).intakeStart())
                .intakeStop()

                // Catalog while moving to first shoot pose, then shoot
                .parallel(p -> p.moveTo(firstToShoot).catalog())
                .shoot()

                // Move through second pickup sequence
                .moveTo(shootToSecondOne)
                .moveTo(shootToSecondTwo)
                .parallel(p -> p.moveTo(shootToSecondThree).intakeStart())
                .intakeStop()

                // Catalog while moving to second shoot pose, then shoot
                .parallel(p -> p.moveTo(secondToShoot).catalog())
                .shoot()

                // Park
                .moveTo(secondShootToStop)
                .build();
    }
}
