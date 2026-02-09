package utility;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.util.PoseHistory;

/**
 * Public field drawing utility for Panels dashboard.
 * Based on the Drawing class in pedroPathing/Tuning.java,
 * made public so it can be used from teleOps and Autos packages.
 * Adds drawTeleOpDebug() for TeleOp pose tracking without a Follower.
 */
public class FieldDrawing {
    public static final double ROBOT_RADIUS = 9;
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotStyle = new Style("", "#3F51B5", 0.75);
    private static final Style trailStyle = new Style("", "#4CAF50", 0.75);
    private static final Style pathStyle  = new Style("", "#FF9800", 0.5);

    /** Must call once during init to set Pedro field offsets. */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }


    /**
     * Draw current path, closest point on path, pose history trail, and robot.
     * Call this each loop in Autonomous.
     */
    public static void drawFollowerDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), pathStyle);
            Pose closestPoint = follower.getPointFromPath(
                    follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(),
                    follower.getCurrentPath().getHeadingGoal(
                            follower.getCurrentPath().getClosestPointTValue())), robotStyle);
        }
        drawPoseHistory(follower.getPoseHistory(), trailStyle);
        drawRobot(follower.getPose(), trailStyle);
        sendPacket();
    }


    /**
     * Draw a pose trail from parallel x/y arrays plus the current robot pose.
     * This is the TeleOp equivalent of drawFollowerDebug.
     *
     * @param xHistory array of x coordinates (oldest to newest)
     * @param yHistory array of y coordinates (oldest to newest)
     * @param count    number of valid entries in the arrays
     * @param current  current robot pose
     */
    // Draw every Nth point when trail is long to reduce overhead
    private static final int TRAIL_SKIP = 4;

    public static void drawTeleOpDebug(double[] xHistory, double[] yHistory,
                                       int count, Pose current) {
        if (count > 1) {
            panelsField.setStyle(trailStyle);
            // When trail is long, skip points to reduce line draw calls
            int step = (count > 50) ? TRAIL_SKIP : 1;
            for (int i = 0; i < count - step; i += step) {
                int next = Math.min(i + step, count - 1);
                panelsField.moveCursor(xHistory[i], yHistory[i]);
                panelsField.line(xHistory[next], yHistory[next]);
            }
        }
        drawRobot(current, trailStyle);
        sendPacket();
    }


    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) ||
                Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2;
        double y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent();
        double y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotStyle);
    }

    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();
        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }
        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);
        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {
            panelsField.moveCursor(
                    poseTracker.getXPositionsArray()[i],
                    poseTracker.getYPositionsArray()[i]);
            panelsField.line(
                    poseTracker.getXPositionsArray()[i + 1],
                    poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    public static void sendPacket() {
        panelsField.update();
    }
}
