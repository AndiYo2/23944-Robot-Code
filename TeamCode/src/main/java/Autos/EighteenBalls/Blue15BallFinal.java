package Autos.EighteenBalls;

import Autos.AutonTemplate;
import Constants.EnumConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;


@Autonomous(name = "Blue15BallFinal")
public class Blue15BallFinal extends AutonTemplate {
    public static double maxSpeed = 1;

    private PathChain shootToCorner, cornerToShoot,
            shootToSecond, secondToGate, gateToShoot,
            shootToFirst, firstToShoot,
            shootToThird, thirdToShoot,
            shootToEnd;

    // Start pose
    private final Pose startPose = new Pose(55.500, 6.750, Math.toRadians(90));

    // Shoot positions
    private final Pose shootPose = new Pose(54.500, 14.000, Math.toRadians(135));
    private final Pose upperShootPose = new Pose(48.000, 85.000, Math.toRadians(135));
    private final Pose upperShootPose135 = new Pose(48.000, 85.000, Math.toRadians(135));
    private final Pose thirdShootPose = new Pose(57.000, 15.000, Math.toRadians(135));

    // Corner positions
    private final Pose cornerPrepPose = new Pose(24.500, 8.500, Math.toRadians(180));
    private final Pose cornerPose = new Pose(11.000, 8.500, Math.toRadians(180));

    // Second spike
    private final Pose secondPrepPose = new Pose(39.000, 60.000, Math.toRadians(180));
    private final Pose secondPose = new Pose(17.000, 60.000, Math.toRadians(180));

    // Gate
    private final Pose gatePose = new Pose(17.500, 66.00, Math.toRadians(180));

    // First spike
    private final Pose firstSpikePose = new Pose(19.000, 85.000, Math.toRadians(180));

    // Third spike
    private final Pose thirdSpikePose = new Pose(18.500, 36.000, Math.toRadians(180));

    // End pose
    private final Pose endPose = new Pose(55.000, 21.000, Math.toRadians(135));

    // Control points
    private final Pose shootToCornerControl = new Pose(37.000, 14.500);
    private final Pose shootToSecondControl = new Pose(54.000, 50.500);
    private final Pose secondToGateControl = new Pose(25.000, 62.500);
    private final Pose gateToShootControl = new Pose(48.000, 66.500);
    private final Pose shootToThirdControl1 = new Pose(49.500, 35.000);
    private final Pose shootToThirdControl2 = new Pose(45.500, 34.500);

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        shootToCorner = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, shootToCornerControl, cornerPrepPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), cornerPrepPose.getHeading())
                .addPath(new BezierLine(cornerPrepPose, cornerPose))
                .setLinearHeadingInterpolation(cornerPrepPose.getHeading(), cornerPose.getHeading())
                .build();

        cornerToShoot = follower.pathBuilder()
                .addPath(new BezierLine(cornerPose, shootPose))
                .setLinearHeadingInterpolation(cornerPose.getHeading(), shootPose.getHeading())
                .build();

        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, shootToSecondControl, secondPrepPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondPrepPose.getHeading())
                .addPath(new BezierLine(secondPrepPose, secondPose))
                .setLinearHeadingInterpolation(secondPrepPose.getHeading(), secondPose.getHeading())
                .build();

        secondToGate = follower.pathBuilder()
                .addPath(new BezierCurve(secondPose, secondToGateControl, gatePose))
                .setLinearHeadingInterpolation(secondPose.getHeading(), gatePose.getHeading())
                .build();

        gateToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gateToShootControl, upperShootPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), upperShootPose.getHeading())
                .build();

        shootToFirst = follower.pathBuilder()
                .addPath(new BezierLine(upperShootPose, firstSpikePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), firstSpikePose.getHeading())
                .build();

        firstToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstSpikePose, upperShootPose135))
                .setLinearHeadingInterpolation(firstSpikePose.getHeading(), upperShootPose135.getHeading())
                .build();

        shootToThird = follower.pathBuilder()
                .addPath(new BezierCurve(upperShootPose135, shootToThirdControl1, shootToThirdControl2, thirdSpikePose))
                .setLinearHeadingInterpolation(Math.toRadians(180), thirdSpikePose.getHeading())
                .build();

        thirdToShoot = follower.pathBuilder()
                .addPath(new BezierLine(thirdSpikePose, thirdShootPose))
                .setLinearHeadingInterpolation(thirdSpikePose.getHeading(), thirdShootPose.getHeading())
                .build();

        shootToEnd = follower.pathBuilder()
                .addPath(new BezierLine(thirdShootPose, endPose))
                .setLinearHeadingInterpolation(thirdShootPose.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        super.init();
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Blue;
        Constants.SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .parallel(p -> p.delay(.7).limelightScan())
                .shoot()
                .intakeStart()
                .moveTo(shootToCorner, maxSpeed, false)
                .delay(.4)
                .parallel(p -> p.moveTo(cornerToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                .setSpindexerMode(EnumConstants.ShootingMode.Sorted)
                .intakeStart()
                .moveTo(shootToSecond, .8, false)
                .moveTo(secondToGate, maxSpeed, false)
                .intakeStop()
                .delay(1.5)
                .parallel(p -> p.moveTo(gateToShoot, maxSpeed, false).guaranteeSortedAutoCatalog())
                .slowShoot()
                .intakeStart()
                .moveTo(shootToFirst, maxSpeed, false)
                .parallel(p -> p.moveTo(firstToShoot, maxSpeed, false).guaranteeSortedAutoCatalog())
                .slowShoot()
                .intakeStart()
                .moveTo(shootToThird, maxSpeed, false)
                .parallel(p -> p.moveTo(thirdToShoot, maxSpeed, false).guaranteeSortedAutoCatalog())
                .slowShoot()
                .moveTo(shootToEnd, maxSpeed, false)
                .build();
    }
}
