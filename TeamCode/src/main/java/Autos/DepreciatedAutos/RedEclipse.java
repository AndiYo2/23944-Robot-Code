package Autos.DepreciatedAutos;

import Autos.AutonTemplate;
import Constants.EnumConstants;
import Constants.SpindexerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import commands.CommandSequenceBuilder;

@Disabled
@Autonomous(name = "RedEclipse")
public class RedEclipse extends AutonTemplate {
    public static double maxSpeed = 1;
    private PathChain shootToFirst, firstToShoot, shootToSecond, secondToShoot, shootToThird, shootToFourth, shootToFifth, shootToStop;

    // Start pose
    private final Pose startPose = new Pose(87.800, 8.000, Math.toRadians(90));

    // Start curve control point
    private final Pose startCurveControl = new Pose(91.000, 17.000);

    // Second pickup (upper row)
    private final Pose secondCurveControl = new Pose(97.500, 39.000);
    private final Pose secondPickupPose = new Pose(130.000, 34.000, Math.toRadians(0));

    // Shoot position
    private final Pose shootPose = new Pose(89.000, 16.000, Math.toRadians(30));
    private final Pose postShootPose = new Pose(89.000, 16.000, Math.toRadians(0));

    // Cycle prep & pickup
    private final Pose cyclePrepPose = new Pose(105.000, 11.500, Math.toRadians(0));
    private final Pose cyclePrepPoseAngled = new Pose(105.000, 11.500, Math.toRadians(-10));
    private final Pose firstCyclePickupPose = new Pose(131.000, 11.500, Math.toRadians(0));
    private final Pose cyclePickupPose = new Pose(130.000, 11.500, Math.toRadians(0));

    // Return to shoot control point
    private final Pose returnToShootControl = new Pose(111.000, 19.000);

    // Fourth cycle waypoints
    private final Pose fourthPrepPose = new Pose(105.000, 11.000, Math.toRadians(0));
    private final Pose fourthCurveControl1 = new Pose(130.000, 10.000);
    private final Pose fourthWaypoint1 = new Pose(130.000, 25.500, Math.toRadians(90));
    private final Pose fourthCurveControl2 = new Pose(136.500, 32.500);
    private final Pose fourthWaypoint2 = new Pose(134.000, 47.500, Math.toRadians(90));

    // End pose
    private final Pose stopPose = new Pose(94.500, 20.500, Math.toRadians(45));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        // Cycle 1 outbound (fluid)
        shootToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, startCurveControl, cyclePrepPose))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .addPath(new BezierLine(cyclePrepPose, firstCyclePickupPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Cycle 1 return
        firstToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstCyclePickupPose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                .build();

        // Cycle 2 outbound (upper row)
        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(postShootPose, secondCurveControl, secondPickupPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Cycle 2 return (curve)
        secondToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(secondPickupPose, returnToShootControl, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                .build();

        // Cycle 3 outbound (fluid)
        shootToThird = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, cyclePrepPoseAngled))
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(-10))
                .addPath(new BezierLine(cyclePrepPoseAngled, cyclePickupPose))
                .setLinearHeadingInterpolation(Math.toRadians(-10), Math.toRadians(0))
                .build();

        // Cycle 4 outbound: fluid collect path (line + curve + curve)
        shootToFourth = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, fourthPrepPose))
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                .addPath(new BezierCurve(fourthPrepPose, fourthCurveControl1, fourthWaypoint1))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .addPath(new BezierCurve(fourthWaypoint1, fourthCurveControl2, fourthWaypoint2))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        // Cycle 5 outbound: duplicate of cycle 4
        shootToFifth = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, fourthPrepPose))
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                .addPath(new BezierCurve(fourthPrepPose, fourthCurveControl1, fourthWaypoint1))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .addPath(new BezierCurve(fourthWaypoint1, fourthCurveControl2, fourthWaypoint2))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        // Park
        shootToStop = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, stopPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), stopPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        super.init();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .delay(.35)
                .shoot()
                .intakeStart()
                .moveTo(shootToFirst, maxSpeed, false)
                .delay(.75)
                .parallel(p -> p.moveTo(firstToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                .intakeStart()
                .moveTo(shootToSecond, maxSpeed, false)
                .delay(.15)
                .parallel(p -> p.moveTo(secondToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                .intakeStart()
                .ballCollectMoveToAndCatalog(shootToThird, shootPose, .8, false, .3)
                .shoot()
                .intakeStart()
                .ballCollectMoveToAndCatalog(shootToFourth, shootPose, maxSpeed, false, .3)
                .shoot()
                .intakeStart()
                .ballCollectMoveToAndCatalog(shootToFifth, shootPose, maxSpeed, false, .3)
                .shoot()
                .moveTo(shootToStop, maxSpeed, false)
                .build();
    }
}
