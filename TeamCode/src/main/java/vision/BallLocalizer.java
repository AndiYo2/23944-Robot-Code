package vision;

import com.pedropathing.geometry.Pose;
import Constants.EnumConstants.BallColor;

/**
 * Static utility class for transforming ball detections from camera frame
 * to field frame via the robot pose.
 *
 * Known v1 limitation: no lens undistortion is applied. The FTC SDK pipeline
 * doesn't expose undistortion cleanly; the error is acceptable for lane-level
 * decisions at typical detection distances (24-60").
 */
public class BallLocalizer {

    private BallLocalizer() {} // static utility

    /**
     * Transforms a camera-frame detection to a field-frame ball position.
     *
     * Camera frame: +Z_cam = forward from lens, +X_cam = camera's right.
     * Robot frame:  +X_robot = forward, +Y_robot = left (Pedro convention).
     * Field frame:  Pedro field coordinates with heading CCW from +X_field.
     *
     * @param d         raw detection with cameraFwd (Z_cam) and cameraLat (X_cam)
     * @param robotPose robot pose at scan time (field frame, heading in radians)
     * @return field-frame ball position
     */
    public static FieldBall toFieldFrame(Detection d, Pose robotPose) {
        // Step 1: Camera frame -> Robot frame
        // Camera is at (CAM_OFFSET_X, CAM_OFFSET_Y) in robot frame, pointing along +X_robot.
        // Z_cam maps to +X_robot, X_cam (right) maps to -Y_robot (left-is-positive).
        double xRobot = d.cameraFwd + VisionConstants.CAM_OFFSET_X;
        double yRobot = -d.cameraLat + VisionConstants.CAM_OFFSET_Y;

        // Step 2: Robot frame -> Field frame
        // Rotate by robot heading (CCW from +X_field).
        double heading = robotPose.getHeading();
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        double xField = robotPose.getX() + xRobot * cosH - yRobot * sinH;
        double yField = robotPose.getY() + xRobot * sinH + yRobot * cosH;

        return new FieldBall(xField, yField, d.color, d.confidence, 1);
    }
}
