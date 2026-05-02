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


@Autonomous(name = "BlueBackCyclerCornerFirst")
public class BlueBackCyclerCornerFirst extends AutonTemplate {
    public static double maxSpeed = 1;

    private PathChain startToCorner, cornerToShoot,
            startToThirdSpike, thirdSpikeToShoot,
            cycle1Go, cycle1Return,
            cycle2Go, cycle2Return,
            cycle2LastGo, cycle2LastReturn,
            shootToEnd;

    // Start pose
    private final Pose startPose = new Pose(55.500, 6.750, Math.toRadians(90));

    // Corner-first approach
    private final Pose cornerApproachControl = new Pose(45.000, 14.000);
    private final Pose cornerPrepPose = new Pose(36.500, 9.500, Math.toRadians(180));
    private final Pose cornerPose = new Pose(11.000, 9.500, Math.toRadians(180));

    // Corner-first return shoot pose
    private final Pose cornerShootControl = new Pose(33.000, 13.500);
    private final Pose cornerShootPose = new Pose(54.000, 11.500, Math.toRadians(145));

    // Shoot positions
    private final Pose shootPose = new Pose(54.000, 11.500, Math.toRadians(145));
    private final Pose shootPose180 = new Pose(54.000, 11.500, Math.toRadians(175));

    // Third spike
    private final Pose thirdSpikePrepControl = new Pose(54.500, 36.000);
    private final Pose thirdSpikePrepPose = new Pose(40.000, 36.000, Math.toRadians(180));
    private final Pose thirdSpikePose = new Pose(17.000, 36.000, Math.toRadians(180));

    // Cycle corners
    private final Pose cycle1MidPose = new Pose(36.500, 9.500, Math.toRadians(180));
    private final Pose cycle1Pose = new Pose(12.500, 9.500, Math.toRadians(180));
    private final Pose cycle2Pose = new Pose(11.000, 22.500, Math.toRadians(180));
    private final Pose cycle2LastPose = new Pose(11.000, 32.500, Math.toRadians(180));

    // End pose
    private final Pose endPose = new Pose(49.000, 17.500, Math.toRadians(135));

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
                .addPath(new BezierCurve(cornerPose, cornerShootControl, cornerShootPose))
                .setLinearHeadingInterpolation(cornerPose.getHeading(), cornerShootPose.getHeading())
                .build();

        startToThirdSpike = follower.pathBuilder()
                .addPath(new BezierCurve(cornerShootPose, thirdSpikePrepControl, thirdSpikePrepPose))
                .setLinearHeadingInterpolation(cornerShootPose.getHeading(), thirdSpikePrepPose.getHeading())
                .addPath(new BezierLine(thirdSpikePrepPose, thirdSpikePose))
                .setLinearHeadingInterpolation(thirdSpikePrepPose.getHeading(), thirdSpikePose.getHeading())
                .build();

        thirdSpikeToShoot = follower.pathBuilder()
                .addPath(new BezierLine(thirdSpikePose, shootPose))
                .setLinearHeadingInterpolation(thirdSpikePose.getHeading(), shootPose.getHeading())
                .build();

        cycle1Go = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, cycle1MidPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), cycle1MidPose.getHeading())
                .addPath(new BezierLine(cycle1MidPose, cycle1Pose))
                .setLinearHeadingInterpolation(cycle1MidPose.getHeading(), cycle1Pose.getHeading())
                .build();

        cycle1Return = follower.pathBuilder()
                .addPath(new BezierLine(cycle1Pose, shootPose))
                .setLinearHeadingInterpolation(cycle1Pose.getHeading(), shootPose.getHeading())
                .build();

        cycle2Go = follower.pathBuilder()
                .addPath(new BezierLine(shootPose180, cycle2Pose))
                .setLinearHeadingInterpolation(shootPose180.getHeading(), cycle2Pose.getHeading())
                .build();

        cycle2Return = follower.pathBuilder()
                .addPath(new BezierLine(cycle2Pose, shootPose))
                .setLinearHeadingInterpolation(cycle2Pose.getHeading(), shootPose.getHeading())
                .build();

        cycle2LastGo = follower.pathBuilder()
                .addPath(new BezierLine(shootPose180, cycle2LastPose))
                .setLinearHeadingInterpolation(shootPose180.getHeading(), cycle2LastPose.getHeading())
                .build();

        cycle2LastReturn = follower.pathBuilder()
                .addPath(new BezierLine(cycle2LastPose, shootPose))
                .setLinearHeadingInterpolation(cycle2LastPose.getHeading(), shootPose.getHeading())
                .build();

        shootToEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        super.init();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Blue;

        CommandSequenceBuilder builder = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .delay(.9)
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
        for (int i = 0; i < 3; i++) {
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

        // Final sweep before the end-grab: extended cycle2 (Y +10)
        builder
                .intakeStart()
                .moveToAndCollect(cycle2LastGo, maxSpeed, 0.7, 0.6)
                .parallel(p -> p.moveTo(cycle2LastReturn, maxSpeed, false).autoCatalog())
                .shoot();

        autonomousCommand = builder
                .intakeStart()
                .moveToAndCollect(cycle1Go, maxSpeed, 0.7, 0.6)
                .build();
    }
}
