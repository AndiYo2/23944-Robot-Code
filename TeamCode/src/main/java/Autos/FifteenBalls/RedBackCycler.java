package Autos.FifteenBalls;

import Autos.AutonTemplate;
import Constants.EnumConstants;
import Constants.SpindexerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;


@Autonomous(name = "RedBackCycler")
public class RedBackCycler extends AutonTemplate {
    public static double maxSpeed = 1;
    private PathChain shootToFirst, firstToShoot, shootToSecond, secondToShoot, shootToThird, thirdToShoot, shootToFourth, fourthToShoot, shootToFifth, fifthToShoot, shootToStop;

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
    private final Pose firstCyclePickupPose = new Pose(131.500, 11.500, Math.toRadians(0));
    private final Pose cyclePickupPose = new Pose(131.000, 11.500, Math.toRadians(0));

    // Return to shoot control point
    private final Pose returnToShootControl = new Pose(111.000, 19.000);

    // Fourth pickup
    private final Pose shootToFourthControl = new Pose(114.500, 48.500);
    private final Pose fourthPickupPose = new Pose(130.000, 25.000, Math.toRadians(-30));

    // End pose
    private final Pose stopPose = new Pose(94.500, 20.500, Math.toRadians(30));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        shootToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, startCurveControl, cyclePrepPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), cyclePrepPose.getHeading())
                .addPath(new BezierLine(cyclePrepPose, firstCyclePickupPose))
                .setLinearHeadingInterpolation(cyclePrepPose.getHeading(), firstCyclePickupPose.getHeading())
                .build();

        firstToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstCyclePickupPose, shootPose))
                .setLinearHeadingInterpolation(firstCyclePickupPose.getHeading(), shootPose.getHeading())
                .build();

        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(postShootPose, secondCurveControl, secondPickupPose))
                .setLinearHeadingInterpolation(postShootPose.getHeading(), secondPickupPose.getHeading())
                .build();

        secondToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(secondPickupPose, returnToShootControl, shootPose))
                .setLinearHeadingInterpolation(secondPickupPose.getHeading(), shootPose.getHeading())
                .build();

        shootToThird = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, cyclePrepPoseAngled))
                .setLinearHeadingInterpolation(shootPose.getHeading(), cyclePrepPoseAngled.getHeading())
                .addPath(new BezierLine(cyclePrepPoseAngled, cyclePickupPose))
                .setLinearHeadingInterpolation(cyclePrepPoseAngled.getHeading(), cyclePickupPose.getHeading())
                .build();

        thirdToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(cyclePickupPose, returnToShootControl, shootPose))
                .setLinearHeadingInterpolation(cyclePickupPose.getHeading(), shootPose.getHeading())
                .build();

        shootToFourth = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, shootToFourthControl, fourthPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), fourthPickupPose.getHeading())
                .build();

        fourthToShoot = follower.pathBuilder()
                .addPath(new BezierLine(fourthPickupPose, shootPose))
                .setLinearHeadingInterpolation(fourthPickupPose.getHeading(), shootPose.getHeading())
                .build();

        shootToFifth = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, cyclePrepPoseAngled))
                .setLinearHeadingInterpolation(shootPose.getHeading(), cyclePrepPoseAngled.getHeading())
                .addPath(new BezierLine(cyclePrepPoseAngled, cyclePickupPose))
                .setLinearHeadingInterpolation(cyclePrepPoseAngled.getHeading(), Math.toRadians(-10))
                .build();

        fifthToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(cyclePickupPose, returnToShootControl, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(-10), shootPose.getHeading())
                .build();

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
                .delay(.55)
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