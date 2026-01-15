package Autos.NineBalls;

import Autos.AutonTemplate;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import framework.AutonSequence;
import Constants.EnumConstants;
import Constants.RobotConstants;

@Configurable
@Autonomous(name = "BlueBack9Ball", group = "NineBall")
public class BlueBack9Ball extends AutonTemplate {
    public static double maxSpeed = 1;
    private PathChain shootToFirst, firstToShoot, shootToSecond, secondToShoot, shootToStop;

    private final Pose startPose = new Pose(56.5, 8.5, Math.toRadians(90));

    private final Pose firstPickupPose = new Pose(22.5, 38.0, Math.toRadians(180));
    private final Pose firstPickupControlPoint = new Pose(57.5, 30.0);
    private final Pose firstShootPose = new Pose(54.5, 15, Math.toRadians(90));
    private final Pose secondPickupPose = new Pose(22.5, 61.0, Math.toRadians(180));
    private final Pose secondPickupControlPoint = new Pose(62.8, 60.0);
    private final Pose secondShootPose = new Pose(54.5, 15, Math.toRadians(90));
    private final Pose stopPose = new Pose(56.5, 40.5, Math.toRadians(90));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);


        shootToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, firstPickupControlPoint, firstPickupPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        firstToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstPickupPose, firstShootPose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), firstShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(firstShootPose, secondPickupControlPoint, secondPickupPose))
                .setLinearHeadingInterpolation(firstShootPose.getHeading(), secondPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        secondToShoot = follower.pathBuilder()
                .addPath(new BezierLine(secondPickupPose, secondShootPose))
                .setLinearHeadingInterpolation(secondPickupPose.getHeading(), secondShootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToStop = follower.pathBuilder()
                .addPath(new BezierLine(secondShootPose, stopPose))
                .setLinearHeadingInterpolation(secondShootPose.getHeading(), stopPose.getHeading())
                .setGlobalDeceleration()
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
    }

    @Override
    public void init() {
        super.init();
        RobotConstants.Robot.allianceColor = EnumConstants.AllianceColor.Blue;
        executor = new AutonSequence(follower, intake, catalogManager, sequenceManager, limelight)
                .parallel(p -> p.limelightScan().catalog())
                .shoot()
                .parallel(p -> p.moveTo(shootToFirst, maxSpeed).intakeStart())
                .intakeStop()
                .parallel(p -> p.moveTo(firstToShoot, maxSpeed).catalog())
                .shoot()
                .parallel(p -> p.moveTo(shootToSecond, maxSpeed).intakeStart())
                .intakeStop()
                .parallel(p -> p.moveTo(secondToShoot, maxSpeed).catalog())
                .shoot()
                .moveTo(shootToStop, maxSpeed)
                .build();
    }
}
