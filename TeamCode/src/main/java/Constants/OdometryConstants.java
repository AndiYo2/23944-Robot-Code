package Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Odometry, field positions, and goal constants.
 */
@Configurable
public class OdometryConstants {
    public static Pose blueStartPoint = new Pose(56.5, 7.5, Math.toRadians(90)); // Test out (55.5, 6.75)
    public static Pose redStartPoint = new Pose(87.5, 7.5, Math.toRadians(90)); // Test out (88.5, 6.75,

    public static Pose standardStartPoint = blueStartPoint;

    public static double yawScalar = .998148;


    public static double BLUE_GOAL_X = 3.0;
    public static double BLUE_GOAL_Y = 140.0;
    public static double RED_GOAL_X = 140.0;
    public static double RED_GOAL_Y = 140.0;

    public static Pose redGatePose = new Pose(131, 59, Math.toRadians(31.5));
    public static Pose blueGatePose = new Pose(13, 59, Math.toRadians(148.5));

    public static Pose endingAutonPose;

    /** Converts a Pedro Pose to FTC Pose2D for the Pinpoint odometry computer. */
    public static Pose2D toPose2D(Pose pose) {
        return new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());
    }
}
