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


@Autonomous(name = "BlueBackCycler")
public class BlueBackCycler extends AutonTemplate {
    public static double maxSpeed = 1;
    private PathChain shootToFirst, firstToShoot, shootToSecond, secondToShoot, shootToThird, thirdToShoot, shootToFourth, fourthToShoot, shootToFifth, fifthToShoot, shootToStop;

    // Start pose
    private final Pose startPose = new Pose(56.200, 8.000, Math.toRadians(90));

    // Start curve control point
    private final Pose startCurveControl = new Pose(53.000, 17.000);

    // Second pickup (upper row)
    private final Pose secondCurveControl = new Pose(50.000, 40.000);
    private final Pose secondPickupPose = new Pose(14.000, 37.000, Math.toRadians(180));

    // Shoot position
    private final Pose shootPose = new Pose(55.000, 16.000, Math.toRadians(150));
    private final Pose postShootPose = new Pose(55.000, 16.000, Math.toRadians(180));

    // Cycle prep & pickup
    private final Pose cyclePrepPose = new Pose(39.000, 11.500, Math.toRadians(180));
    private final Pose cyclePrepPoseAngled = new Pose(39.000, 11.500, Math.toRadians(190));
    private final Pose firstCyclePickupPose = new Pose(13.000, 11.500, Math.toRadians(180));
    private final Pose cyclePickupPose = new Pose(14.000, 11.500, Math.toRadians(180));

    // Return to shoot control point
    private final Pose returnToShootControl = new Pose(33.000, 19.000);

    // Fourth pickup
    private final Pose fourthPickupPose = new Pose(14.000, 23.000, Math.toRadians(180));

    // End pose
    private final Pose stopPose = new Pose(49.500, 20.500, Math.toRadians(150));

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
                .addPath(new BezierLine(postShootPose, fourthPickupPose))
                .setLinearHeadingInterpolation(postShootPose.getHeading(), fourthPickupPose.getHeading())
                .build();

        fourthToShoot = follower.pathBuilder()
                .addPath(new BezierLine(fourthPickupPose, shootPose))
                .setLinearHeadingInterpolation(fourthPickupPose.getHeading(), shootPose.getHeading())
                .build();

        shootToFifth = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, cyclePrepPoseAngled))
                .setLinearHeadingInterpolation(shootPose.getHeading(), cyclePrepPoseAngled.getHeading())
                .addPath(new BezierLine(cyclePrepPoseAngled, cyclePickupPose))
                .setLinearHeadingInterpolation(cyclePrepPoseAngled.getHeading(), cyclePickupPose.getHeading())
                .build();

        fifthToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(cyclePickupPose, returnToShootControl, shootPose))
                .setLinearHeadingInterpolation(cyclePickupPose.getHeading(), shootPose.getHeading())
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
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Blue;

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
                .moveTo(shootToThird, maxSpeed, false)
                .delay(.55)
                .parallel(p -> p.moveTo(thirdToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                .intakeStart()
                .moveTo(shootToFourth, maxSpeed, false)
                .delay(.55)
                .parallel(p -> p.moveTo(fourthToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                .intakeStart()
                .moveTo(shootToFifth, maxSpeed, false)
                .delay(.55)
                .parallel(p -> p.moveTo(fifthToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                .moveTo(shootToStop, maxSpeed, false)
                .build();
    }
}