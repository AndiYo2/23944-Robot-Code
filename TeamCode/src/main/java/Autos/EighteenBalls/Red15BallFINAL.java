package Autos.EighteenBalls;

import Autos.AutonTemplate;
import Constants.EnumConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;


@Autonomous(name = "RedSorted")
public class Red15BallFINAL extends AutonTemplate {
    public static double maxSpeed = 1;

    private PathChain shootToCorner, cornerToShoot,
            shootToSecond, secondToGate, gateToShoot,
            shootToFirst, firstToShoot,
            shootToThird, thirdToShoot;

    // Start pose
    private final Pose startPose = new Pose(88.500, 6.750, Math.toRadians(90));

    // Shoot positions
    private final Pose shootPose = new Pose(88.500, 13.500, Math.toRadians(30));
    private final Pose upperShootPose = new Pose(86.000, 82.000, Math.toRadians(30));
    private final Pose thirdShootPose = new Pose(87.000, 108.000, Math.toRadians(30));

    // Corner positions
    private final Pose cornerPrepPose = new Pose(119.500, 8.500, Math.toRadians(0));
    private final Pose cornerPose = new Pose(133.000, 8.500, Math.toRadians(0));

    // Second spike
    private final Pose secondPrepPose = new Pose(99.000, 57.500, Math.toRadians(0));
    private final Pose secondPose = new Pose(131.500, 57.500, Math.toRadians(0));

    // Gate
    private final Pose gatePose = new Pose(127.0, 63.500, Math.toRadians(0));

    // First spike
    private final Pose firstSpikePose = new Pose(125.000, 82.000, Math.toRadians(0));

    // Third spike
    private final Pose thirdPrepPose = new Pose(98.500, 33.000, Math.toRadians(0));
    private final Pose thirdSpikePose = new Pose(131.500, 33.000, Math.toRadians(0));

    // Control points
    private final Pose shootToCornerControl = new Pose(100.000, 17.000);
    private final Pose cornerToShootControl = new Pose(109.000, 15.000);
    private final Pose shootToSecondControl = new Pose(86.745, 52.766);
    private final Pose secondToGateControl = new Pose(118.500, 62.500);
    private final Pose gateToShootControl = new Pose(96.500, 65.000);
    private final Pose shootToThirdControl = new Pose(75.500, 35.500);

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
                .addPath(new BezierCurve(cornerPose, cornerToShootControl, shootPose))
                .setLinearHeadingInterpolation(cornerPose.getHeading(), shootPose.getHeading())
                .build();

        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, shootToSecondControl, secondPrepPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondPrepPose.getHeading())
                .addPath(new BezierLine(secondPrepPose, secondPose))
                .setTangentHeadingInterpolation()
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
                .setLinearHeadingInterpolation(Math.toRadians(0), firstSpikePose.getHeading())
                .build();

        firstToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstSpikePose, upperShootPose))
                .setLinearHeadingInterpolation(firstSpikePose.getHeading(), upperShootPose.getHeading())
                .build();

        shootToThird = follower.pathBuilder()
                .addPath(new BezierCurve(upperShootPose, shootToThirdControl, thirdPrepPose))
                .setLinearHeadingInterpolation(upperShootPose.getHeading(), thirdPrepPose.getHeading())
                .addPath(new BezierLine(thirdPrepPose, thirdSpikePose))
                .setLinearHeadingInterpolation(thirdPrepPose.getHeading(), thirdSpikePose.getHeading())
                .build();

        thirdToShoot = follower.pathBuilder()
                .addPath(new BezierLine(thirdSpikePose, thirdShootPose))
                .setLinearHeadingInterpolation(thirdSpikePose.getHeading(), thirdShootPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        super.init();
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;
        Constants.SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .parallel(p -> p.delay(.9).limelightScan())
                .delay(.2)
                .shoot()
                .intakeStart()
                .moveTo(shootToCorner, maxSpeed, false)
                .delay(.4)
                .parallel(p -> p.moveTo(cornerToShoot, maxSpeed, false).autoCatalog())
                .delay(.2)
                .shoot()
                .setSpindexerMode(EnumConstants.ShootingMode.Sorted)
                .intakeStart()
                .moveTo(shootToSecond, maxSpeed, false)
                .moveTo(secondToGate, .8, false)
                .intakeStop()
                .delay(1)
                .parallel(p -> p.moveTo(gateToShoot, maxSpeed, false).guaranteeSortedAutoCatalog())
                .delay(.2)
                .slowShoot()
                .intakeStart()
                .moveTo(shootToFirst, maxSpeed, false)
                .parallel(p -> p.moveTo(firstToShoot, maxSpeed, false).guaranteeSortedAutoCatalog())
                .delay(.2)
                .slowShoot()
                .intakeStart()
                .moveTo(shootToThird, maxSpeed, false)
                .parallel(p -> p.moveTo(thirdToShoot, maxSpeed, false).guaranteeSortedAutoCatalog())
                .delay(.2)
                .slowShoot()
                .build();
    }
}
