package Autos;

import Autos.AutonTemplate;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import framework.AutonSequence;
import utility.RobotConstants;
import utility.RobotHardware;

@Autonomous(name = "BlueFront12Ball")
public class BlueFront12Ball extends AutonTemplate {
    public static double maxSpeed = .8;
    private PathChain startToShoot, shootToFirst, firstToShoot, gateUnload, shootToSecond, secondToShoot, shootToThird, thirdToShoot, shootToStop;

    // Named pose constants (matching BlueBack9Ball pattern)
    private final Pose startPose = new Pose(21, 122.5, Math.toRadians(54));
    private final Pose shootPose = new Pose(54, 90, Math.toRadians(135));
    private final Pose firstPickupPose = new Pose(28, 86, Math.toRadians(180));
    private final Pose firstPickupControlPoint = new Pose(66.5, 78.5);
    private final Pose secondPickupPose = new Pose(22.5, 59, Math.toRadians(180));
    private final Pose secondPickupControlPoint = new Pose(69.7, 49.5);

    private final Pose gatePose = new Pose(17, 69.5, Math.toRadians(180));
    private final Pose gateControlPose = new Pose(19, 72, Math.toRadians(180));
    private final Pose secondShootControlPoint = new Pose(58, 60.5);
    private final Pose thirdPickupPose = new Pose(26, 36, Math.toRadians(180));
    private final Pose thirdPickupControlPoint = new Pose(71.5, 30.5);
    private final Pose stopPose = new Pose(32, 85, Math.toRadians(180));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        shootToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, secondPickupControlPoint, secondPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondPickupPose.getHeading())
                .build();

        gateUnload = follower.pathBuilder()
                .addPath(new BezierCurve(secondPickupPose, gateControlPose, gatePose))
                .setLinearHeadingInterpolation(secondPickupPose.getHeading(), gatePose.getHeading())
                .build();

        firstToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, secondShootControlPoint, shootPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose.getHeading())
                .build();


        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, firstPickupControlPoint, firstPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstPickupPose.getHeading())
                .build();

        secondToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstPickupPose, shootPose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), shootPose.getHeading())
                .build();

        shootToThird = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, thirdPickupControlPoint, thirdPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), thirdPickupPose.getHeading())
                .build();

        thirdToShoot = follower.pathBuilder()
                .addPath(new BezierLine(thirdPickupPose, shootPose))
                .setLinearHeadingInterpolation(thirdPickupPose.getHeading(), shootPose.getHeading())
                .build();

        shootToStop = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, stopPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), stopPose.getHeading())
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        // Not used - framework handles execution
    }

    @Override
    public void init() {
        super.init();
        RobotConstants.UpdatableConstants.allianceColor = RobotConstants.Enums.AllianceColor.Blue;

        executor = new AutonSequence(follower, intake, catalogManager, sequenceManager, limelight)
                .parallel(p -> p.moveTo(startToShoot, maxSpeed).limelightScan().catalog())
                .shoot()
                .parallel(p -> p.moveTo(shootToFirst, maxSpeed).intakeStart())
                .delay(.1)
                .intakeStop()
                .moveTo(gateUnload, maxSpeed)
                .delay(.3)
                .parallel(p -> p.moveTo(firstToShoot, maxSpeed).catalog())
                .shoot()
                .parallel(p -> p.moveTo(shootToSecond, maxSpeed).intakeStart())
                .parallel(p -> p.moveTo(secondToShoot, maxSpeed).catalog())
                .intakeStop()
                .shoot()
                .parallel(p -> p.moveTo(shootToThird, maxSpeed).intakeStart())
                .parallel(p -> p.moveTo(thirdToShoot, maxSpeed).catalog())
                .intakeStop()
                .shoot()
                .moveTo(shootToStop, maxSpeed)
                .build();
    }
}