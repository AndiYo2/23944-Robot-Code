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
            cycleGo, cycleReturn,
            shootToEnd;

    // Start pose
    private final Pose startPose = new Pose(55.500, 6.750, Math.toRadians(90));

    // Shoot positions
    private final Pose shootPose = new Pose(54.000, 10.500, Math.toRadians(150));

    // Third spike
    private final Pose thirdSpikePrepPose = new Pose(37.000, 27.000, Math.toRadians(135));
    private final Pose thirdSpikePose = new Pose(21.500, 27.500, Math.toRadians(110));

    // Cycle corner
    private final Pose cyclePose = new Pose(11.000, 9.000, Math.toRadians(180));

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

        cycleGo = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, cyclePose))
                .setLinearHeadingInterpolation(cyclePose.getHeading(), cyclePose.getHeading())
                .build();

        cycleReturn = follower.pathBuilder()
                .addPath(new BezierLine(cyclePose, shootPose))
                .setLinearHeadingInterpolation(cyclePose.getHeading(), shootPose.getHeading())
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
                .moveTo(startToThirdSpike, maxSpeed, false)
                .parallel(p -> p.moveTo(thirdSpikeToShoot, maxSpeed, false).autoCatalog())
                .shoot();

        for (int i = 0; i < 5; i++) {
            builder
                    .intakeStart()
                    .moveTo(cycleGo, maxSpeed, false)
                    .delay(.4)
                    .parallel(p -> p.moveTo(cycleReturn, maxSpeed, false).autoCatalog())
                    .shoot();
        }

        autonomousCommand = builder
                .moveTo(shootToEnd, maxSpeed, false)
                .build();
    }
}