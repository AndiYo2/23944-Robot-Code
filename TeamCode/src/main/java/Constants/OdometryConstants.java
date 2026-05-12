package Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@Configurable
public class OdometryConstants {
    public static Pose blueStartPoint = new Pose(55.5, 6.75, Math.toRadians(90)); // Test out (55.5, 6.75)
    public static Pose redStartPoint = new Pose(88.5, 6.75, Math.toRadians(90)); // Test out (88.5, 6.75,

    public static Pose standardStartPoint = blueStartPoint;

    public static double yawScalar = .998148;


    public static double BLUE_GOAL_X = 4.0;
    public static double BLUE_GOAL_Y = 140.0;
    public static double RED_GOAL_X = 140.0;
    public static double RED_GOAL_Y = 140.0;

    public static Pose redGatePose = new Pose(127, 62, Math.toRadians(0));
    public static Pose blueGatePose = new Pose(18, 65, Math.toRadians(180));

    public static Pose redIntakePose = new Pose(132, 41, Math.toRadians(55));
    public static Pose blueIntakePose = new Pose(13, 44.5, Math.toRadians(125));

    public static Pose endingAutonPose;

    public static Pose2D toPose2D(Pose pose) {
        return new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());
    }
}
