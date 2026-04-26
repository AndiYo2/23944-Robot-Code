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
@Autonomous(name = "BlueExodus")
public class BlueExodus extends AutonTemplate {
    public static double maxSpeed = 1;
    private PathChain startToShoot, shootToFirstPrep, firstPrepToFirst, firstToGate, firstGateToShoot, shootToGatePrep1, prepToThree, gateToShootThree, ShootToFourth, fourthToGate, fiveToShoot;

    // Start pose
    private final Pose startPose = new Pose(32.500, 133.500, Math.toRadians(90));

    // Shoot positions
    private final Pose shootPoseStart = new Pose(56.000, 82.000, Math.toRadians(90));
    private final Pose shootPose = new Pose(56.000, 82.000, Math.toRadians(150));
    private final Pose shootPoseThree = new Pose(56.000, 85.000, Math.toRadians(150));
    private final Pose postShootPose = new Pose(56.000, 85.000, Math.toRadians(180));

    // First prep & pickup
    private final Pose firstPrepControl = new Pose(54.000, 67.000);
    private final Pose firstPrepPose = new Pose(43.500, 61.000, Math.toRadians(180));
    private final Pose firstPickupPose = new Pose(17.000, 61.000, Math.toRadians(180));

    // First gate
    private final Pose firstGateControl = new Pose(24.000, 61.000);
    private final Pose firstGatePose = new Pose(16.750, 67.000, Math.toRadians(180));
    private final Pose firstGateToShootControl = new Pose(38.500, 58.500);

    // Gate approach & pickup (shared by 2nd and 4th cycles)
    private final Pose gateApproachControl = new Pose(42.000, 68.500);
    private final Pose gateToPickupControl = new Pose(19.000, 57.500);
    private final Pose gatePickupPose = new Pose(13.50, 55.000, Math.toRadians(180));
    private final Pose pickupToShootControl = new Pose(36.000, 65.000);

    // Second gate
    private final Pose secondGatePose = new Pose(16.800, 64.500, Math.toRadians(180));

    // Fourth pickup & gate
    private final Pose fourthPickupPose = new Pose(16.00, 85.000, Math.toRadians(180));
    private final Pose fourthGateControl = new Pose(25.000, 80.750);
    private final Pose fourthGatePose = new Pose(16.500, 76.500, Math.toRadians(180));

    // End pose
    private final Pose lastShootPose = new Pose(52.500, 118.000, Math.toRadians(150));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPoseStart))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPoseStart.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToFirstPrep = follower.pathBuilder()
                .addPath(new BezierCurve(shootPoseStart, firstPrepControl, firstPrepPose))
                .setLinearHeadingInterpolation(shootPoseStart.getHeading(), firstPrepPose.getHeading())
                .setGlobalDeceleration()
                .build();

        firstPrepToFirst = follower.pathBuilder()
                .addPath(new BezierLine(firstPrepPose, firstPickupPose))
                .setLinearHeadingInterpolation(firstPrepPose.getHeading(), firstPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        firstToGate = follower.pathBuilder()
                .addPath(new BezierCurve(firstPickupPose, firstGateControl, firstGatePose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), firstGatePose.getHeading())
                .setGlobalDeceleration()
                .build();

        firstGateToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(firstGatePose, firstGateToShootControl, shootPose))
                .setLinearHeadingInterpolation(firstGatePose.getHeading(), shootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        shootToGatePrep1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, gateApproachControl, secondGatePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondGatePose.getHeading())
                .setGlobalDeceleration()
                .build();

        prepToThree = follower.pathBuilder()
                .addPath(new BezierCurve(secondGatePose, gateToPickupControl, gatePickupPose))
                .setLinearHeadingInterpolation(secondGatePose.getHeading(), gatePickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        gateToShootThree = follower.pathBuilder()
                .addPath(new BezierCurve(gatePickupPose, pickupToShootControl, shootPoseThree))
                .setLinearHeadingInterpolation(gatePickupPose.getHeading(), shootPoseThree.getHeading())
                .setGlobalDeceleration()
                .build();

        ShootToFourth = follower.pathBuilder()
                .addPath(new BezierLine(postShootPose, fourthPickupPose))
                .setLinearHeadingInterpolation(postShootPose.getHeading(), fourthPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        fourthToGate = follower.pathBuilder()
                .addPath(new BezierCurve(fourthPickupPose, fourthGateControl, fourthGatePose))
                .setLinearHeadingInterpolation(fourthPickupPose.getHeading(), fourthGatePose.getHeading())
                .setGlobalDeceleration()
                .build();

        fiveToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(fourthGatePose, pickupToShootControl, lastShootPose))
                .setLinearHeadingInterpolation(fourthGatePose.getHeading(), lastShootPose.getHeading())
                .setGlobalDeceleration()
                .build();
    }

    @Override
    public void init() {
        super.init();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Blue;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .moveTo(startToShoot, maxSpeed, false)
                .shoot()
                .intakeStart()
                .moveTo(shootToFirstPrep, maxSpeed, false)
                .moveTo(firstPrepToFirst, maxSpeed, false)
                .parallel(p -> p.autoCatalog().moveTo(firstToGate, .8, false))
                .delay(1.2)
                .moveTo(firstGateToShoot, maxSpeed, false)
                .shoot()
                .intakeStart()
                .moveTo(shootToGatePrep1, .8, true)
                .delay(2)
                .moveTo(prepToThree, maxSpeed, false)
                .delay(.75)
                .parallel(p -> p.autoCatalog().moveTo(gateToShootThree, maxSpeed, false))
                .shoot()
                .intakeStart()
                .moveTo(ShootToFourth, maxSpeed, false)
                .moveTo(fourthToGate, .8, false)
                .autoCatalog()
                .delay(.3)
                .moveTo(fiveToShoot, maxSpeed, false)
                .shoot()
                .build();
    }
}