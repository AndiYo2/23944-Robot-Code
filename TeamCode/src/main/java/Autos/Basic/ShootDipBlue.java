package Autos.Basic;

import Autos.AutonTemplate;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;

@Configurable
@Autonomous(name = "ShootDipBlue")
public class ShootDipBlue extends AutonTemplate {
    public static double maxSpeed = .8;
    private PathChain shootToFirst;

    private final Pose startPose = new Pose(56.5, 8.5, Math.toRadians(90));
    private final Pose stopPose = new Pose(30, 10, Math.toRadians(90));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        shootToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, stopPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), stopPose.getHeading())
                .setGlobalDeceleration()
                .build();


    }

    @Override
    public void init() {
        super.init();
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Blue;
        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                .moveTo(shootToFirst, maxSpeed)
                .build();
    }
}
