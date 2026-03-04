package Autos.PartnerAutos;

import Autos.AutonTemplate;
import Constants.EnumConstants;
import Constants.SpindexerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;


@Autonomous(name = "RedExodus")
public class RedExodus extends AutonTemplate {
    public static double maxSpeed = 1;
    private PathChain startToShoot, shootToFirstPrep, firstPrepToFirst, firstToGate, firstGateToShoot, shootToGatePrep1, prepToThree, gateToShootThree, ShootToFourth, fourthToGate, fiveToShoot;

    // Start pose
    private final Pose startPose = new Pose(111.000, 133.500, Math.toRadians(90));

    // Shoot positions
    private final Pose shootPoseStart = new Pose(88.000, 82.000, Math.toRadians(90));
    private final Pose shootPose = new Pose(88.000, 82.000, Math.toRadians(30));
    private final Pose postShootPose = new Pose(88.000, 82.000, Math.toRadians(0));

    // First prep & pickup
    private final Pose firstPrepControl = new Pose(90.000, 67.000);
    private final Pose firstPrepPose = new Pose(100.500, 59.000, Math.toRadians(0));
    private final Pose firstPickupPose = new Pose(127.000, 59.000, Math.toRadians(0));

    // First gate
    private final Pose firstGateControl = new Pose(120.000, 61.000);
    private final Pose firstGatePose = new Pose(127, 67.00, Math.toRadians(0));
    private final Pose firstGateToShootControl = new Pose(105.500, 58.500);

    // Gate approach & pickup (shared by 2nd and 4th cycles)
    private final Pose gateApproachControl = new Pose(102.000, 68.500);
    private final Pose gateToPickupControl = new Pose(125.000, 57.500);
    private final Pose gatePickupPose = new Pose(132.00, 55.000, Math.toRadians(0));
    private final Pose pickupToShootControl = new Pose(108.000, 65.000);

    // Second gate
    private final Pose secondGatePose = new Pose(127.250, 64.500, Math.toRadians(0));

    // Third pickup & gate
    private final Pose thirdPickupPose = new Pose(128.00, 80.000, Math.toRadians(0));

    // End pose
    private final Pose lastShootPose = new Pose(91.500, 118.000, Math.toRadians(30));

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
                .addPath(new BezierCurve(gatePickupPose, pickupToShootControl, shootPose))
                .setLinearHeadingInterpolation(gatePickupPose.getHeading(), shootPose.getHeading())
                .setGlobalDeceleration()
                .build();

        ShootToFourth = follower.pathBuilder()
                .addPath(new BezierLine(postShootPose, thirdPickupPose))
                .setLinearHeadingInterpolation(postShootPose.getHeading(), thirdPickupPose.getHeading())
                .setGlobalDeceleration()
                .build();

        fiveToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(thirdPickupPose, pickupToShootControl, lastShootPose))
                .setLinearHeadingInterpolation(thirdPickupPose.getHeading(), lastShootPose.getHeading())
                .setGlobalDeceleration()
                .build();
    }

    @Override
    public void init() {
        super.init();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;

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
                .moveTo(ShootToFourth, .9, false)
                .autoCatalog()
                .delay(.45)
                .moveTo(fiveToShoot, maxSpeed, false)
                .shoot()
                .build();
    }
}