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


@Autonomous(name = "RedBackCyclerCornerFirst")
public class RedBackCyclerCornerFirst extends AutonTemplate {
    public static double maxSpeed = 1;

    private PathChain startToCorner, cornerToShoot,
            startToThirdSpike, thirdSpikeToShoot,
            cycle1Go, cycle1Return,
            cycle2Go, cycle2Return,
            shootToEnd;

    // Start pose
    private final Pose startPose = new Pose(88.500, 6.750, Math.toRadians(90));

    // Corner-first approach
    private final Pose cornerApproachControl = new Pose(103.500, 13.500);
    private final Pose cornerPrepPose = new Pose(114.500, 9.000, Math.toRadians(0));
    private final Pose cornerPose = new Pose(133.000, 9.000, Math.toRadians(0));

    // Corner-first return shoot pose (Path10 end)
    private final Pose cornerShootPose = new Pose(93.000, 11.500, Math.toRadians(45));

    // Shoot positions
    private final Pose shootPose = new Pose(90.000, 11.500, Math.toRadians(45));
    private final Pose shootPose30 = new Pose(90.000, 11.500, Math.toRadians(30));

    // Third spike
    private final Pose thirdSpikePrepPose = new Pose(106.500, 21.500, Math.toRadians(45));
    private final Pose thirdSpikePose = new Pose(122.500, 27.500, Math.toRadians(45));

    // Cycle corners
    private final Pose cycle1Pose = new Pose(133.000, 9.000, Math.toRadians(0));
    private final Pose cycle2Pose = new Pose(133.000, 22.500, Math.toRadians(0));

    // End pose
    private final Pose endPose = new Pose(98.500, 15.500, Math.toRadians(30));

    // Control points
    private final Pose thirdSpikeControl = new Pose(112.000, 27.500);
    private final Pose cycle1GoControl = new Pose(111.500, 11.000);
    private final Pose cycle1ReturnControl = new Pose(110.500, 13.000);

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        startToCorner = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, cornerApproachControl, cornerPrepPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), cornerPrepPose.getHeading())
                .addPath(new BezierLine(cornerPrepPose, cornerPose))
                .setLinearHeadingInterpolation(cornerPrepPose.getHeading(), cornerPose.getHeading())
                .build();

        cornerToShoot = follower.pathBuilder()
                .addPath(new BezierLine(cornerPose, cornerShootPose))
                .setLinearHeadingInterpolation(cornerPose.getHeading(), cornerShootPose.getHeading())
                .build();

        startToThirdSpike = follower.pathBuilder()
                .addPath(new BezierLine(cornerShootPose, thirdSpikePrepPose))
                .setLinearHeadingInterpolation(cornerShootPose.getHeading(), thirdSpikePrepPose.getHeading())
                .addPath(new BezierCurve(thirdSpikePrepPose, thirdSpikeControl, thirdSpikePose))
                .setLinearHeadingInterpolation(thirdSpikePrepPose.getHeading(), thirdSpikePose.getHeading())
                .build();

        thirdSpikeToShoot = follower.pathBuilder()
                .addPath(new BezierLine(thirdSpikePose, shootPose))
                .setLinearHeadingInterpolation(thirdSpikePose.getHeading(), shootPose.getHeading())
                .build();

        cycle1Go = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, cycle1GoControl, cycle1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        cycle1Return = follower.pathBuilder()
                .addPath(new BezierCurve(cycle1Pose, cycle1ReturnControl, shootPose30))
                .setLinearHeadingInterpolation(cycle1Pose.getHeading(), shootPose30.getHeading())
                .build();

        cycle2Go = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, cycle2Pose))
                .setLinearHeadingInterpolation(Math.toRadians(0), cycle2Pose.getHeading())
                .build();

        cycle2Return = follower.pathBuilder()
                .addPath(new BezierLine(cycle2Pose, shootPose30))
                .setLinearHeadingInterpolation(cycle2Pose.getHeading(), shootPose30.getHeading())
                .build();

        shootToEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose30, endPose))
                .setLinearHeadingInterpolation(shootPose30.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        super.init();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;

        CommandSequenceBuilder builder = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .delay(.7)
                .shoot()
                .intakeStart()
                .moveToAndCollect(startToCorner, maxSpeed, 0.7, 0.6)
                .parallel(p -> p.moveTo(cornerToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                .intakeStart()
                .moveToAndCollect(startToThirdSpike, maxSpeed, 0.7, 0.6)
                .parallel(p -> p.moveTo(thirdSpikeToShoot, maxSpeed, false).autoCatalog())
                .shoot();

        boolean useCycle1 = true;
        for (int i = 0; i < 4; i++) {
            if (useCycle1) {
                builder
                        .intakeStart()
                        .moveToAndCollect(cycle1Go, maxSpeed, 0.7, 0.6)
                        .parallel(p -> p.moveTo(cycle1Return, maxSpeed, false).autoCatalog())
                        .shoot();
            } else {
                builder
                        .intakeStart()
                        .moveToAndCollect(cycle2Go, maxSpeed, 0.7, 0.6)
                        .parallel(p -> p.moveTo(cycle2Return, maxSpeed, false).autoCatalog())
                        .shoot();
            }
            useCycle1 = !useCycle1;
        }

        autonomousCommand = builder
                .intakeStart()
                .moveToAndCollect(cycle1Go, maxSpeed, 0.7, 0.6)
                .build();
    }
}
