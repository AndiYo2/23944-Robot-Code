package Autos.NineBalls;

import Autos.AutonTemplate;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import framework.AutonSequence;

@Autonomous(name = "RedFront9Ball", group = "NineBall")
public class RedFront9Ball extends AutonTemplate {
    private PathChain startToShoot, shootToFirst, firstToShoot, shootToSecond, secondToShoot, shootToStop;

    private final Pose startPose = new Pose(123.5, 122.0, Math.toRadians(126));
    private final Pose shootPose = new Pose(90.0, 90.0, Math.toRadians(45));
    private final Pose firstPickupPose = new Pose(120.5, 82.0, Math.toRadians(0));
    private final Pose firstPickupControlPoint = new Pose(85.5, 76.0);
    private final Pose secondPickupPose = new Pose(120.5, 57.5, Math.toRadians(0));
    private final Pose secondPickupControlPoint = new Pose(82.0, 53.0);
    private final Pose stopPose = new Pose(116.0, 72.0, Math.toRadians(0));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        shootToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, firstPickupControlPoint, firstPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstPickupPose.getHeading())
                .build();

        firstToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstPickupPose, shootPose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), shootPose.getHeading())
                .build();

        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, secondPickupControlPoint, secondPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondPickupPose.getHeading())
                .build();

        secondToShoot = follower.pathBuilder()
                .addPath(new BezierLine(secondPickupPose, shootPose))
                .setLinearHeadingInterpolation(secondPickupPose.getHeading(), shootPose.getHeading())
                .build();

        shootToStop = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, stopPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), stopPose.getHeading())
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
    }

    @Override
    public void init() {
        super.init();
        executor = new AutonSequence(follower, intake, catalogManager, sequenceManager, limelight)
                .parallel(p -> p.moveTo(startToShoot).limelightScan().catalog())
                .shoot()
                .parallel(p -> p.moveTo(shootToFirst).intakeStart())
                .intakeStop()
                .parallel(p -> p.moveTo(firstToShoot).catalog())
                .shoot()
                .parallel(p -> p.moveTo(shootToSecond).intakeStart())
                .intakeStop()
                .parallel(p -> p.moveTo(secondToShoot).catalog())
                .shoot()
                .moveTo(shootToStop)
                .build();
    }
}
