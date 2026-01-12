package Autos.Basic;

import Autos.AutonTemplate;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import framework.AutonSequence;

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
                .build();


    }

    @Override
    protected void autonomousPathUpdate() {
    }

    @Override
    public void init() {
        super.init();
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Blue;
        executor = new AutonSequence(follower, intake, catalogManager, sequenceManager, limelight)
                .moveTo(shootToFirst, maxSpeed)
                .build();
    }
}
