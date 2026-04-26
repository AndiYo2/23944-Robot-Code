package Autos.PartnerAutos;

import Autos.AutonTemplate;
import Constants.EnumConstants;
import Constants.SpindexerConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;


@Autonomous(name = "BlueBackCycler")
public class BlueBackCycler extends AutonTemplate {
    public static double maxSpeed = 1;

    private PathChain startToThirdSpike, thirdSpikeToShoot,
            cycle1Go, cycle1Return,
            cycle2Go, cycle2Return,
            shootToEnd;

    // Start pose
    private final Pose startPose = new Pose(55.500, 6.750, Math.toRadians(90));

    // Shoot positions
    private final Pose shootPose = new Pose(54.000, 11.500, Math.toRadians(150));
    private final Pose shootPose180 = new Pose(54.000, 11.500, Math.toRadians(180));

    // Third spike
    private final Pose thirdSpikePrepPose = new Pose(37.000, 24.000, Math.toRadians(135));
    private final Pose thirdSpikePose = new Pose(21.500, 24.500, Math.toRadians(110));

    // Cycle corners
    private final Pose cycle1Pose = new Pose(11.000, 9.000, Math.toRadians(180));
    private final Pose cycle2Pose = new Pose(11.000, 22.500, Math.toRadians(180));

    // End pose
    private final Pose endPose = new Pose(49.000, 17.500, Math.toRadians(135));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        startToThirdSpike = follower.pathBuilder()
                .addPath(new BezierLine(startPose, thirdSpikePrepPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), thirdSpikePrepPose.getHeading())
                .addPath(new BezierLine(thirdSpikePrepPose, thirdSpikePose))
                .setLinearHeadingInterpolation(thirdSpikePrepPose.getHeading(), thirdSpikePose.getHeading())
                .build();

        thirdSpikeToShoot = follower.pathBuilder()
                .addPath(new BezierLine(thirdSpikePose, shootPose))
                .setLinearHeadingInterpolation(thirdSpikePose.getHeading(), shootPose.getHeading())
                .build();

        cycle1Go = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, cycle1Pose))
                .setLinearHeadingInterpolation(cycle1Pose.getHeading(), cycle1Pose.getHeading())
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
                .setLinearHeadingInterpolation(Math.toRadians(0), shootPose.getHeading())
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
                .delay(.7)
                .shoot()
                .intakeStart()
                .moveToAndCollect(startToThirdSpike, maxSpeed, 0.7, 0.6)
                .parallel(p -> p.moveTo(thirdSpikeToShoot, maxSpeed, false).autoCatalog())
                .shoot();

        boolean useCycle1 = true;
        for (int i = 0; i < 5; i++) {
            if (useCycle1) {
                builder
                        .intakeStart()
                        .moveToAndCollect(cycle1Go, maxSpeed, 0.7, 0.6)
                        .delay(.4)
                        .parallel(p -> p.moveTo(cycle1Return, maxSpeed, false).autoCatalog())
                        .shoot();
            } else {
                builder
                        .intakeStart()
                        .moveToAndCollect(cycle2Go, maxSpeed, 0.7, 0.6)
                        .delay(.4)
                        .parallel(p -> p.moveTo(cycle2Return, maxSpeed, false).autoCatalog())
                        .shoot();
            }
            useCycle1 = !useCycle1;
        }

        autonomousCommand = builder
                .intakeStart()
                .moveToAndCollect(cycle1Go, maxSpeed)
                .build();
    }
}