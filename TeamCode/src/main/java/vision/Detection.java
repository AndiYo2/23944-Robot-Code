package vision;

import Constants.EnumConstants.BallColor;

/**
 * Raw detection from a single frame's ColorBlobLocatorProcessor output.
 * Positions are in camera frame (pinhole model, inches).
 */
public class Detection {
    public final BallColor color;
    public final double pixelCenterX;
    public final double pixelCenterY;
    public final double pixelRadius;
    public final double cameraFwd;   // Z_cam: forward distance from camera (inches)
    public final double cameraLat;   // X_cam: lateral offset, + = camera's right (inches)
    public final double confidence;  // 0-1, from 0.5*circularity + 0.5*fillRatio

    public Detection(BallColor color, double pixelCenterX, double pixelCenterY,
                     double pixelRadius, double cameraFwd, double cameraLat, double confidence) {
        this.color = color;
        this.pixelCenterX = pixelCenterX;
        this.pixelCenterY = pixelCenterY;
        this.pixelRadius = pixelRadius;
        this.cameraFwd = cameraFwd;
        this.cameraLat = cameraLat;
        this.confidence = confidence;
    }
}
