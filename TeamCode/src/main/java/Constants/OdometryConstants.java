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
    public static Pose2D blueStartPoint = new Pose2D(DistanceUnit.INCH, 56.5, 8.5, AngleUnit.DEGREES, 90);
    public static Pose2D redStartPoint = new Pose2D(DistanceUnit.INCH, 87.5, 8.5, AngleUnit.DEGREES, 90);

    // Default start point (Blue alliance by default)
    public static Pose2D standardStartPoint = blueStartPoint;

    public static double yawScalar = .998148;

    public static Pose redParkZone = new Pose(38.75, 32.5, Math.toRadians(90));
    public static Pose blueParkZone = new Pose(105.25, 32.5, Math.toRadians(90));


    public static double BLUE_GOAL_X = 0.0;
    public static double BLUE_GOAL_Y = 141.0;
    public static double RED_GOAL_X = 136.0;
    public static double RED_GOAL_Y = 141.0;

    public static Pose endingAutonPose;
}
