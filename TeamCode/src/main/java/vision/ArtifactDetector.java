package vision;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
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

    /** No-op — processors stay enabled to avoid onDrawFrame null race condition in the SDK. */
    public void enable() {
    }

    /** No-op — processors stay enabled to avoid onDrawFrame null race condition in the SDK. */
    public void disable() {
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

            // SDK computes circularity internally (4*PI*area / perimeter^2).
            // We still keep this as a shape sanity check even though it's
            // biased low when the top/bottom of the ball is shadowed — any
            // vaguely round blob still passes the 0.35 threshold.
            double circularity = blob.getCircularity();
            if (circularity < VisionConstants.MIN_CIRCULARITY) continue;

            // Horizontal-only silhouette measurement. The bottom of the ball
            // is often shadowed and the top has a specular highlight, so the
            // contour's vertical extent is unreliable. The horizontal extent
            // (leftmost to rightmost pixel) is where the ball's surface is
            // tangent to the camera's line of sight, reflectance is stable
            // from both lighting directions, and the silhouette edge stays
            // crisp. Bounding rect is axis-aligned (NOT minAreaRect).
            MatOfPoint2f contour2f = blob.getContourAsFloat();
            Rect box = Imgproc.boundingRect(contour2f);
            double pixelWidth = box.width;
            double midX       = box.x + box.width  / 2.0;
            double midY       = box.y + box.height / 2.0;

            if (pixelWidth < VisionConstants.MIN_BALL_PIXEL_DIAMETER) continue;

            // Confidence uses the axis-aligned bounding box area as the
            // reference instead of minEnclosingCircle's area. Fill ratio is
            // higher for a well-formed ball silhouette, lower for noisy
            // ring/arc artifacts.
            double boxArea   = (double) box.width * box.height;
            double fillRatio = (double) area / Math.max(boxArea, 1);
            double confidence = 0.5 * circularity + 0.5 * fillRatio;
            if (confidence < VisionConstants.MIN_CONFIDENCE) continue;

            // Pinhole model using horizontal diameter ONLY.
            //   zCam = fx * BALL_DIAMETER / pixelWidth           (forward distance)
            //   xCam = (midX - cx) * zCam / fx                   (lateral offset)
            double zCam = scaledFx * VisionConstants.BALL_DIAMETER_INCHES / pixelWidth;
            double xCam = (midX - scaledCx) * zCam / scaledFx;

            // pixelRadius is stored as half the horizontal width so NMS uses
            // the same metric as the distance calculation.
            double pixelRadius = pixelWidth / 2.0;
            out.add(new Detection(color, midX, midY, pixelRadius,
                    zCam, xCam, confidence));
        }
    }

    /**
     * Suppress inner detections only when they look like the same object seen
     * twice: the smaller detection must be WELL inside the larger one (center
     * within 0.6 × larger radius) AND similar size (radius ratio > 0.6). Two
     * adjacent equal-sized balls now both survive — the old rule wrongly ate
     * one of them.
     */
    private void suppressInnerBlobs(List<Detection> detections) {
        detections.sort(Comparator.comparingDouble((Detection d) -> d.pixelRadius).reversed());

        List<Detection> kept = new ArrayList<>();
        for (Detection det : detections) {
            boolean suppressed = false;
            for (Detection k : kept) {
                double dx = det.pixelCenterX - k.pixelCenterX;
                double dy = det.pixelCenterY - k.pixelCenterY;
                double centerDist = Math.sqrt(dx * dx + dy * dy);
                boolean inside = centerDist < 0.6 * k.pixelRadius;
                double radiusRatio = det.pixelRadius / Math.max(k.pixelRadius, 1.0);
                boolean similarSize = radiusRatio > 0.6;
                if (inside && similarSize) {
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
        detections.sort(Comparator.comparingDouble((Detection d) -> d.confidence).reversed());
    }
}
