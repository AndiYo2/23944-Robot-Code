package Autos.FifteenBalls;

import Autos.AutonTemplate;
import Constants.SpindexerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;
import Constants.EnumConstants;
import Constants.RobotConstants;

@Autonomous(name = "BlueFifteenBall", group = "FifteenBalll")
public class BlueFifteenBall extends AutonTemplate {
    public static double maxSpeed = 1;
    private PathChain shootToFirstOne, shootToFirstTwo, shootToFirstThree, firstToShoot, shootToSecond, secondToGate,
            gateToShoot, shootToThird, thirdToShoot, shootToFourth, fourthToShoot, shootToStop;

    // ===== POSES =====
    private final Pose startPose = new Pose(56.5, 8.5, Math.toRadians(90));

    private final Pose firstPickupPose1 = new Pose(15.0, 12.75, Math.toRadians(195));

    private final Pose firstPickupPose2 = new Pose(13.3, 8.7, Math.toRadians(180));
    private final Pose firstPickupPose3 = new Pose(11.0, 8.5, Math.toRadians(180));

    private final Pose shootPose1 = new Pose(52.0, 12.5, Math.toRadians(115));

    private final Pose secondPickupPose = new Pose(19.5, 61.0, Math.toRadians(180));

    private final Pose gatePose = new Pose(16.0, 70.0, Math.toRadians(180));

    private final Pose shootPose2 = new Pose(58.5, 83.5, Math.toRadians(130));

    private final Pose thirdPickupPose = new Pose(21.0, 83.5, Math.toRadians(180));

    private final Pose fourthPickupPose = new Pose(18.5, 37.0, Math.toRadians(180));

    private final Pose endPose = new Pose(51.5, 32.5, Math.toRadians(90));

    // ===== CONTROL POINTS =====
    private final Pose shootToFirstControlPoint = new Pose(44.0, 19.5);
    private final Pose firstToShootControlPoint = new Pose(32.5, 16.5);
    private final Pose shootToSecondControlPoint = new Pose(54.0, 66.0);
    private final Pose secondToGateControlPoint = new Pose(21.0, 69.0);
    private final Pose gateToShootControlPoint = new Pose(44.5, 61.0);
    private final Pose shootToFourthControlPoint = new Pose(70.0, 32.5);

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
    public void init() {
        super.init();
        RobotConstants.Robot.allianceColor = EnumConstants.AllianceColor.Blue;
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        // TODO: Complete the sequence with all paths
        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .parallel(p -> p.limelightScan().catalog())
                .shoot()
                .parallel(p -> p.moveTo(shootToFirstOne, .8).intakeStart())
                .moveTo(shootToFirstTwo, .4)
                .moveTo(shootToFirstThree, .4)
                .delay(1)
                .intakeStop()
                .parallel(p -> p.moveTo(firstToShoot, maxSpeed).catalog())
                .shoot()
                .setSpindexerMode(EnumConstants.ShootingMode.Sorted)
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