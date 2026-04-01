package Autos.EighteenBalls;

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
@Autonomous(name = "Red18Ball")
public class Red18Ball extends AutonTemplate {
    public static double maxSpeed = .85;
    private PathChain StartToShoot, ShootToFirstSpike, FirstSpikeToShoot, ShootToSecondSpike,
            SecondSpikeToGate, SecondSpikeGateToShoot, ShootToGateOpenForThird,
            GateOpenForThirdToThird, ThirdToShoot, ShootToGateOpenForFourth,
            GateOpenForFourthToFourth, FourthToShoot, ShootToFifthSpikePrep,
            FifthSpikePrepToFifthSpike, FifthSpikeToShoot;

    // Named pose constants
    private final Pose startPose = new Pose(111.000, 133.500, Math.toRadians(90));
    private final Pose shootPose = new Pose(120.000, 107.500, Math.toRadians(90));
    private final Pose firstSpikePose = new Pose(120.000, 90.000, Math.toRadians(270));
    private final Pose mainShootPose = new Pose(87.500, 81.000, Math.toRadians(30));
    private final Pose secondSpikeControlPoint = new Pose(116.500, 85.000);
    private final Pose secondSpikePose = new Pose(120.000, 66.000, Math.toRadians(270));
    private final Pose gatePose = new Pose(128.000, 65.000, Math.toRadians(0));
    private final Pose gateInnerControlPoint = new Pose(123.500, 59.000);
    private final Pose thirdFourthPickupPose = new Pose(133.000, 58.000, Math.toRadians(0));
    private final Pose fifthSpikePrepPose = new Pose(120.000, 50.000, Math.toRadians(270));
    private final Pose fifthSpikePose = new Pose(120.000, 41.500, Math.toRadians(270));
    private final Pose lastShootPose = new Pose(88.000, 110.500, Math.toRadians(30));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        StartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        ShootToFirstSpike = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, firstSpikePose))
                .setConstantHeadingInterpolation(firstSpikePose.getHeading())
                .build();

        FirstSpikeToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstSpikePose, mainShootPose))
                .setLinearHeadingInterpolation(firstSpikePose.getHeading(), mainShootPose.getHeading())
                .build();

        ShootToSecondSpike = follower.pathBuilder()
                .addPath(new BezierCurve(mainShootPose, secondSpikeControlPoint, secondSpikePose))
                .setLinearHeadingInterpolation(mainShootPose.getHeading(), secondSpikePose.getHeading())
                .build();

        SecondSpikeToGate = follower.pathBuilder()
                .addPath(new BezierLine(secondSpikePose, gatePose))
                .setLinearHeadingInterpolation(secondSpikePose.getHeading(), secondSpikePose.getHeading())
                .build();

        SecondSpikeGateToShoot = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, mainShootPose))
                .setLinearHeadingInterpolation(secondSpikePose.getHeading(), mainShootPose.getHeading())
                .build();

        ShootToGateOpenForThird = follower.pathBuilder()
                .addPath(new BezierLine(mainShootPose, gatePose))
                .setLinearHeadingInterpolation(mainShootPose.getHeading(), gatePose.getHeading())
                .build();

        GateOpenForThirdToThird = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gateInnerControlPoint, thirdFourthPickupPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), thirdFourthPickupPose.getHeading())
                .build();

        ThirdToShoot = follower.pathBuilder()
                .addPath(new BezierLine(thirdFourthPickupPose, mainShootPose))
                .setLinearHeadingInterpolation(thirdFourthPickupPose.getHeading(), mainShootPose.getHeading())
                .build();

        ShootToGateOpenForFourth = follower.pathBuilder()
                .addPath(new BezierLine(mainShootPose, gatePose))
                .setLinearHeadingInterpolation(mainShootPose.getHeading(), gatePose.getHeading())
                .build();

        GateOpenForFourthToFourth = follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gateInnerControlPoint, thirdFourthPickupPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), thirdFourthPickupPose.getHeading())
                .build();

        FourthToShoot = follower.pathBuilder()
                .addPath(new BezierLine(thirdFourthPickupPose, mainShootPose))
                .setLinearHeadingInterpolation(thirdFourthPickupPose.getHeading(), mainShootPose.getHeading())
                .build();

        ShootToFifthSpikePrep = follower.pathBuilder()
                .addPath(new BezierLine(mainShootPose, fifthSpikePrepPose))
                .setLinearHeadingInterpolation(mainShootPose.getHeading(), fifthSpikePrepPose.getHeading())
                .build();

        FifthSpikePrepToFifthSpike = follower.pathBuilder()
                .addPath(new BezierLine(fifthSpikePrepPose, fifthSpikePose))
                .setLinearHeadingInterpolation(fifthSpikePrepPose.getHeading(), fifthSpikePose.getHeading())
                .build();

        FifthSpikeToShoot = follower.pathBuilder()
                .addPath(new BezierLine(fifthSpikePose, lastShootPose))
                .setLinearHeadingInterpolation(fifthSpikePose.getHeading(), lastShootPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        super.init();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .delay(.75)
                .moveTo(StartToShoot, maxSpeed, false)
                .shoot()
                .intakeStart()
                .moveTo(ShootToFirstSpike, maxSpeed, false)
                .delay(1)
                .autoCatalog()
                .moveTo(FirstSpikeToShoot, maxSpeed, false)
                .shoot()
                .intakeStart()
                .moveTo(ShootToSecondSpike, maxSpeed, false)
                .delay(1)
                .autoCatalog()
                .moveTo(SecondSpikeToGate, maxSpeed, false)
                .moveTo(SecondSpikeGateToShoot, maxSpeed, false)
                .shoot()
                .intakeStart()
                .moveTo(ShootToGateOpenForThird, maxSpeed, false)
                .delay(.3)
                .moveTo(GateOpenForThirdToThird, maxSpeed, false)
                .delay(.3)
                .autoCatalog()
                .moveTo(ThirdToShoot, maxSpeed, false)
                .shoot()
                .intakeStart()
                .moveTo(ShootToGateOpenForFourth, maxSpeed, false)
                .delay(.3)
                .moveTo(GateOpenForFourthToFourth, maxSpeed, false)
                .delay(.3)
                .autoCatalog()
                .moveTo(FourthToShoot, maxSpeed, false)
                .shoot()
                .intakeStart()
                .moveTo(ShootToFifthSpikePrep, maxSpeed, false)
                .moveTo(FifthSpikePrepToFifthSpike, maxSpeed, false)
                .delay(1)
                .autoCatalog()
                .moveTo(FifthSpikeToShoot, maxSpeed, false)
                .shoot()
                .build();
    }
}