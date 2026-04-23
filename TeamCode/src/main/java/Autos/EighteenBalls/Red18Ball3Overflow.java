package Autos.EighteenBalls;

import Autos.AutonTemplate;
import Constants.EnumConstants;
import Constants.SpindexerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;


@Autonomous(name = "Red18Ball3Overflow")
public class Red18Ball3Overflow extends AutonTemplate {
    public static double maxSpeed = 1;
    private PathChain shootToFirst, firstToShoot,
            shootToSecond, secondToGate, gateToShoot,
            shootToThird, thirdToShoot,
            shootToFourth, fourthToShoot,
            shootToFifth, fifthToShoot,
            shootToEnd;

    // Start pose
    private final Pose startPose = new Pose(88.500, 6.750, Math.toRadians(90));

    // Shoot positions
    private final Pose shootPose1 = new Pose(88.500, 13.500, Math.toRadians(35));
    private final Pose shootPose2 = new Pose(89.000, 83.000, Math.toRadians(30));
    // Same XY as shootPose2, heading faces outbound for cycle 3
    private final Pose postShootPose2 = new Pose(89.000, 83.000, Math.toRadians(0));
    private final Pose shootPose3 = new Pose(89.000, 83.000, Math.toRadians(0));
    private final Pose shootPose4 = new Pose(88.500, 12.000, Math.toRadians(35));
    private final Pose shootPose5 = new Pose(79.000, 15.000, Math.toRadians(35));

    // Pickup / waypoint poses
    private final Pose firstPickupPrep = new Pose(119.500, 8.500, Math.toRadians(0));
    private final Pose firstPickupPose = new Pose(131.000, 8.500, Math.toRadians(0));
    private final Pose secondPickupWaypoint = new Pose(99.000, 58.500, Math.toRadians(0));
    private final Pose secondPickupPose = new Pose(131.500, 58.500, Math.toRadians(0));
    private final Pose gatePose = new Pose(126.000, 64.500, Math.toRadians(0));
    private final Pose thirdPickupPose = new Pose(125.000, 83.000, Math.toRadians(0));
    private final Pose fourthPickupPrep = new Pose(98.500, 35.000, Math.toRadians(0));
    private final Pose fourthPickupPose = new Pose(131.500, 35.000, Math.toRadians(0));
    private final Pose fifthWaypointPose = new Pose(100.000, 9.000, Math.toRadians(0));
    private final Pose fifthPickupPose = new Pose(131.000, 8.500, Math.toRadians(0));

    // End / park pose
    private final Pose endPose = new Pose(93.000, 23.000, Math.toRadians(30));

    // Control points
    private final Pose firstCycleControl = new Pose(106.276, 15.060);
    private final Pose firstReturnControl = new Pose(109.000, 15.000);
    private final Pose secondOutboundControl = new Pose(86.745, 52.766);
    private final Pose gateControl = new Pose(118.842, 60.104);
    private final Pose secondReturnControl = new Pose(99.353, 67.853);
    private final Pose fourthOutboundControl = new Pose(75.500, 35.500);

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        // Cycle 1 outbound: start curve into first pickup prep, then line to pickup
        shootToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, firstCycleControl, firstPickupPrep))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstPickupPrep.getHeading())
                .addPath(new BezierLine(firstPickupPrep, firstPickupPose))
                .setLinearHeadingInterpolation(firstPickupPrep.getHeading(), firstPickupPose.getHeading())
                .build();

        // Cycle 1 return
        firstToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(firstPickupPose, firstReturnControl, shootPose1))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), shootPose1.getHeading())
                .build();

        // Cycle 2 outbound: curve up to waypoint, line out to second pickup
        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, secondOutboundControl, secondPickupWaypoint))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), secondPickupWaypoint.getHeading())
                .addPath(new BezierLine(secondPickupWaypoint, secondPickupPose))
                .setLinearHeadingInterpolation(secondPickupWaypoint.getHeading(), secondPickupPose.getHeading())
                .build();

        // Cycle 2 return (split): pickup to gate, then gate to shoot
        secondToGate = follower.pathBuilder()
                .addPath(new BezierCurve(secondPickupPose, gateControl, gatePose))
                .setLinearHeadingInterpolation(secondPickupPose.getHeading(), gatePose.getHeading())
                .build();

        gateToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, secondReturnControl, shootPose2))
                .setLinearHeadingInterpolation(gatePose.getHeading(), shootPose2.getHeading())
                .build();

        // Cycle 3 outbound
        shootToThird = follower.pathBuilder()
                .addPath(new BezierLine(postShootPose2, thirdPickupPose))
                .setLinearHeadingInterpolation(postShootPose2.getHeading(), thirdPickupPose.getHeading())
                .build();

        // Cycle 3 return
        thirdToShoot = follower.pathBuilder()
                .addPath(new BezierLine(thirdPickupPose, shootPose3))
                .setLinearHeadingInterpolation(thirdPickupPose.getHeading(), shootPose3.getHeading())
                .build();

        // Cycle 4 outbound: curve down to prep, line to fourth pickup
        shootToFourth = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose3, fourthOutboundControl, fourthPickupPrep))
                .setLinearHeadingInterpolation(shootPose3.getHeading(), fourthPickupPrep.getHeading())
                .addPath(new BezierLine(fourthPickupPrep, fourthPickupPose))
                .setLinearHeadingInterpolation(fourthPickupPrep.getHeading(), fourthPickupPose.getHeading())
                .build();

        // Cycle 4 return
        fourthToShoot = follower.pathBuilder()
                .addPath(new BezierLine(fourthPickupPose, shootPose4))
                .setLinearHeadingInterpolation(fourthPickupPose.getHeading(), shootPose4.getHeading())
                .build();

        // Cycle 5 outbound (overflow): line through waypoint to fifth pickup
        shootToFifth = follower.pathBuilder()
                .addPath(new BezierLine(shootPose4, fifthWaypointPose))
                .setLinearHeadingInterpolation(shootPose4.getHeading(), fifthWaypointPose.getHeading())
                .addPath(new BezierLine(fifthWaypointPose, fifthPickupPose))
                .setLinearHeadingInterpolation(fifthWaypointPose.getHeading(), fifthPickupPose.getHeading())
                .build();

        // Cycle 5 return
        fifthToShoot = follower.pathBuilder()
                .addPath(new BezierLine(fifthPickupPose, shootPose5))
                .setLinearHeadingInterpolation(fifthPickupPose.getHeading(), shootPose5.getHeading())
                .build();

        // Park
        shootToEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose5, endPose))
                .setLinearHeadingInterpolation(shootPose5.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        super.init();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .delay(.7)
                .shoot()
                // Cycle 1
                .intakeStart()
                .moveTo(shootToFirst, maxSpeed, false)
                .delay(.75)
                .parallel(p -> p.moveTo(firstToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                // Cycle 2
                .setSpindexerMode(EnumConstants.ShootingMode.Sorted)
                .intakeStart()
                .moveTo(shootToSecond, maxSpeed, false)
                .moveTo(secondToGate, maxSpeed, false)
                .delay(.35)
                .parallel(p -> p.moveTo(gateToShoot, maxSpeed, false).guaranteeSortedAutoCatalog())
                .shoot()
                // Cycle 3
                .intakeStart()
                .moveTo(shootToThird, maxSpeed, false)
                .delay(.35)
                .parallel(p -> p.moveTo(thirdToShoot, maxSpeed, false).guaranteeSortedAutoCatalog())
                .shoot()
                // Cycle 4
                .intakeStart()
                .moveTo(shootToFourth, maxSpeed, false)
                .parallel(p -> p.moveTo(fourthToShoot, maxSpeed, false).guaranteeSortedAutoCatalog())
                .shoot()
                // Cycle 5 (overflow)
                .setSpindexerMode(EnumConstants.ShootingMode.Fast)
                .intakeStart()
                .moveTo(shootToFifth, maxSpeed, false)
                .delay(.35)
                .parallel(p -> p.moveTo(fifthToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                // Park
                .moveTo(shootToEnd, maxSpeed, false)
                .build();
    }
}
