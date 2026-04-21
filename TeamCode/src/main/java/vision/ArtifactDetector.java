package vision;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import Constants.EnumConstants.BallColor;
import utility.RobotHardware;

/**
 * Wraps the two ColorBlobLocatorProcessors (green + purple) on the shared VisionPortal.
 * Processors are disabled by default and enabled on-demand during auton scans to avoid
 * wasting CPU cycles when vision isn't needed.
 *
 * Known v1 limitations:
 * - No lens undistortion (acceptable error for lane-level decisions)
 * - Area filter thresholds calibrated at 1920x1080 are scaled to portal resolution
 */
public class ArtifactDetector {
    private final VisionPortal portal;
    private final ColorBlobLocatorProcessor greenProcessor;
    private final ColorBlobLocatorProcessor purpleProcessor;

    // Scaled camera intrinsics for the actual portal resolution
    private final double scaledFx;
    private final double scaledFy;
    private final double scaledCx;
    private final double scaledCy;

    // Scaled area thresholds (calibration was at 1920x1080)
    private final double scaledMinArea;
    private final double scaledMaxArea;

    public ArtifactDetector() {
        RobotHardware hw = RobotHardware.getInstance();
        this.portal = hw.visionPortal;
        this.greenProcessor = hw.greenBlobProcessor;
        this.purpleProcessor = hw.purpleBlobProcessor;

        // Scale intrinsics from calibration resolution to portal resolution
        double scaleX = (double) VisionConstants.VISION_PORTAL_WIDTH / VisionConstants.CALIBRATION_WIDTH;
        double scaleY = (double) VisionConstants.VISION_PORTAL_HEIGHT / VisionConstants.CALIBRATION_HEIGHT;
        scaledFx = VisionConstants.FX * scaleX;
        scaledFy = VisionConstants.FY * scaleY;
        scaledCx = VisionConstants.CX * scaleX;
        scaledCy = VisionConstants.CY * scaleY;

        // Scale area thresholds by pixel-count ratio
        double areaScale = (double) (VisionConstants.VISION_PORTAL_WIDTH * VisionConstants.VISION_PORTAL_HEIGHT)
                / (VisionConstants.CALIBRATION_WIDTH * VisionConstants.CALIBRATION_HEIGHT);
        scaledMinArea = VisionConstants.MIN_CONTOUR_AREA * areaScale;
        scaledMaxArea = VisionConstants.MAX_CONTOUR_AREA * areaScale;
    }

    /** Enable both processors for active scanning. Call before captureFrames. */
    public void enable() {
        greenProcessor.setEnabled(true);
        purpleProcessor.setEnabled(true);
    }

    /** Disable both processors to free CPU when not scanning. */
    public void disable() {
        greenProcessor.setEnabled(false);
        purpleProcessor.setEnabled(false);
    }

    /**
     * Pulls the latest blobs from both processors, applies filtering
     * (area, circularity, aspect, confidence), computes camera-frame
     * position via pinhole model, and suppresses inner blobs.
     *
     * @return filtered detections from the current frame
     */
    /** Debug: raw blob counts from last scanOnce call, before filtering. */
    public int lastGreenBlobCount = 0;
    public int lastPurpleBlobCount = 0;
    public String lastScanError = "";

    public List<Detection> scanOnce() {
        List<Detection> detections = new ArrayList<>();
        lastScanError = "";

        try {
            List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenProcessor.getBlobs();
            lastGreenBlobCount = (greenBlobs != null) ? greenBlobs.size() : -1;
            processBlobs(greenBlobs, BallColor.Green, detections);
        } catch (Exception e) {
            lastScanError += "GREEN:" + e.getClass().getSimpleName() + " ";
        }
        try {
            List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleProcessor.getBlobs();
            lastPurpleBlobCount = (purpleBlobs != null) ? purpleBlobs.size() : -1;
            processBlobs(purpleBlobs, BallColor.Purple, detections);
        } catch (Exception e) {
            lastScanError += "PURPLE:" + e.getClass().getSimpleName() + " ";
        }

        suppressInnerBlobs(detections);
        return detections;
    }

    private void processBlobs(List<ColorBlobLocatorProcessor.Blob> blobs,
                              BallColor color, List<Detection> out) {
        if (blobs == null) return;
        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            int area = blob.getContourArea();
            if (area < scaledMinArea || area > scaledMaxArea) continue;

            if (blob.getAspectRatio() > VisionConstants.MAX_ASPECT_RATIO) continue;

            // SDK computes circularity internally (4*PI*area / perimeter^2)
            double circularity = blob.getCircularity();
            if (circularity < VisionConstants.MIN_CIRCULARITY) continue;

            // Get min enclosing circle from the contour
            // getContourAsFloat() returns MatOfPoint2f directly — no conversion needed
            MatOfPoint2f contour2f = blob.getContourAsFloat();
            Point center = new Point();
            float[] radius = new float[1];
            Imgproc.minEnclosingCircle(contour2f, center, radius);

            double dPixels = 2.0 * radius[0];
            if (dPixels < VisionConstants.MIN_BALL_PIXEL_DIAMETER) continue;

            // Confidence = average of circularity and fill ratio
            double enclosingArea = Math.PI * radius[0] * radius[0];
            double fillRatio = (double) area / Math.max(enclosingArea, 1);
            double confidence = 0.5 * circularity + 0.5 * fillRatio;
            if (confidence < VisionConstants.MIN_CONFIDENCE) continue;

            // Pinhole model: camera-frame position (inches)
            double zCam = scaledFx * VisionConstants.BALL_DIAMETER_INCHES / dPixels;
            double xCam = (center.x - scaledCx) * zCam / scaledFx;

            out.add(new Detection(color, center.x, center.y, radius[0],
                    zCam, xCam, confidence));
        }
    }

    /**
     * Suppress inner detections: balls visible through holes of a closer ball.
     * Sort by radius descending, drop any detection whose center lies inside
     * a larger kept detection's enclosing circle.
     */
    private void suppressInnerBlobs(List<Detection> detections) {
        detections.sort(Comparator.comparingDouble((Detection d) -> d.pixelRadius).reversed());

        List<Detection> kept = new ArrayList<>();
        for (Detection det : detections) {
            boolean suppressed = false;
            for (Detection k : kept) {
                double dx = det.pixelCenterX - k.pixelCenterX;
                double dy = det.pixelCenterY - k.pixelCenterY;
                double dist = Math.sqrt(dx * dx + dy * dy);
                if (dist < k.pixelRadius) {
                    suppressed = true;
                    break;
                }
            }
            if (!suppressed) {
                kept.add(det);
            }
        }

        detections.clear();
        detections.addAll(kept);
        // Sort by confidence descending (matches Python output)
        detections.sort(Comparator.comparingDouble((Detection d) -> d.confidence).reversed());
    }
}
