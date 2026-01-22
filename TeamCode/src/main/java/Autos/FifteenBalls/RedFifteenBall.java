package Autos.FifteenBalls;

import Autos.AutonTemplate;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import framework.AutonSequence;
import Constants.EnumConstants;
import Constants.RobotConstants;

@Autonomous(name = "RedFifteenBall", group = "FifteenBalll")
public class RedFifteenBall extends AutonTemplate {
    public static double maxSpeed = 1;
    private PathChain shootToFirstOne, shootToFirstTwo, shootToFirstThree, firstToShoot, shootToSecond, secondToGate,
            gateToShoot, shootToThird, thirdToShoot, shootToFourth, fourthToShoot, shootToStop;

    // ===== POSES =====
    private final Pose startPose = new Pose(87.5, 8.5, Math.toRadians(90));

    private final Pose firstPickupPose1 = new Pose(129, 12.75, Math.toRadians(345));

    private final Pose firstPickupPose2 = new Pose(130.7, 8.7, Math.toRadians(0));
    private final Pose firstPickupPose3 = new Pose(133, 8.5, Math.toRadians(0));

    private final Pose shootPose1 = new Pose(92.0, 12.5, Math.toRadians(65));

    private final Pose secondPickupPose = new Pose(124.383, 57.585, Math.toRadians(0));

    private final Pose gatePose = new Pose(128.0, 67.0, Math.toRadians(0));

    private final Pose shootPose2 = new Pose(85.5, 81.5, Math.toRadians(50));

    private final Pose thirdPickupPose = new Pose(123.0, 81.5, Math.toRadians(0));

    private final Pose fourthPickupPose = new Pose(125.5, 34.5, Math.toRadians(0));

    private final Pose endPose = new Pose(92.5, 47.5, Math.toRadians(90));

    // ===== CONTROL POINTS =====
    private final Pose shootToFirstControlPoint = new Pose(100.0, 19.5);
    private final Pose firstToShootControlPoint = new Pose(111.5, 16.5);
    private final Pose shootToSecondControlPoint = new Pose(90.0, 61.0);
    private final Pose secondToGateControlPoint = new Pose(122.0, 69.0);
    private final Pose gateToShootControlPoint = new Pose(99.309, 60.947);
    private final Pose shootToFourthControlPoint = new Pose(74.0, 27.0);

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        shootToFirstOne = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, shootToFirstControlPoint, firstPickupPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstPickupPose1.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToFirstTwo = follower.pathBuilder()
                .addPath(new BezierLine(firstPickupPose1, firstPickupPose2))
                .setLinearHeadingInterpolation(firstPickupPose1.getHeading(), firstPickupPose2.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToFirstThree = follower.pathBuilder()
                .addPath(new BezierLine(firstPickupPose2, firstPickupPose3))
                .setLinearHeadingInterpolation(firstPickupPose2.getHeading(), firstPickupPose3.getHeading())
                .setGlobalDeceleration()
                .build();

        firstToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(firstPickupPose3, firstToShootControlPoint, shootPose1))
                .setLinearHeadingInterpolation(firstPickupPose3.getHeading(), shootPose1.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, shootToSecondControlPoint, secondPickupPose))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), secondPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        secondToGate = follower.pathBuilder()
                .addPath(new BezierCurve(secondPickupPose, secondToGateControlPoint, gatePose))
                .setLinearHeadingInterpolation(secondPickupPose.getHeading(), gatePose.getHeading())
                .setGlobalDeceleration()
                .build();

        gateToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gateToShootControlPoint, shootPose2))
                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose2.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToThird = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, thirdPickupPose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), thirdPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        thirdToShoot = follower.pathBuilder()
                .addPath(new BezierLine(thirdPickupPose, shootPose2))
                .setLinearHeadingInterpolation(thirdPickupPose.getHeading(), shootPose2.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToFourth = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, shootToFourthControlPoint, fourthPickupPose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), fourthPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        fourthToShoot = follower.pathBuilder()
                .addPath(new BezierLine(fourthPickupPose, shootPose1))
                .setLinearHeadingInterpolation(fourthPickupPose.getHeading(), shootPose1.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToStop = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1, endPose))
                .setTangentHeadingInterpolation()
                .setGlobalDeceleration()
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
    }

    @Override
    public void init() {
        super.init();
        RobotConstants.Robot.allianceColor = EnumConstants.AllianceColor.Red;
        spindexerManager.setMode(EnumConstants.ShootingMode.Fast);
        // TODO: Complete the sequence with all paths
        executor = new AutonSequence(follower, intake, spindexerManager, limelight)
                .parallel(p -> p.limelightScan().catalog())
                .shoot()
                .parallel(p -> p.moveTo(shootToFirstOne, .8).intakeStart())
                .moveTo(shootToFirstTwo, .4)
                .moveTo(shootToFirstThree, .4)
                .delay(1)
                .intakeStop()
                .parallel(p -> p.moveTo(firstToShoot, maxSpeed).catalog())
                .shoot()
                .parallel(p -> p.moveTo(shootToSecond).intakeStart())
                .moveTo(secondToGate)
                .intakeStop()
                .parallel(p -> p.moveTo(gateToShoot).catalog())
                .shoot()
                .parallel(p -> p.moveTo(shootToThird).intakeStart())
                .intakeStop()
                .parallel(p -> p.moveTo(thirdToShoot).catalog())
                .shoot()
                .parallel(p -> p.moveTo(shootToFourth).intakeStart())
                .intakeStop()
                .parallel(p -> p.moveTo(fourthToShoot).catalog())
                .shoot()
                .moveTo(shootToStop)
                .build();
    }
}
