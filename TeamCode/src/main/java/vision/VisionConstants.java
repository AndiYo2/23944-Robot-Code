package vision;

/**
 * Constants for the vision-based ball detection and lane selection system.
 * Camera intrinsics calibrated at 1920x1080 (john.yml); scaled at runtime
 * to match the actual VisionPortal resolution.
 */
public class VisionConstants {
    // ============ Ball physical ============
    public static final double BALL_DIAMETER_INCHES = 5.0;
    public static final double INTAKE_WIDTH_INCHES = 16.0;

    // ============ Lane definitions (FIELD coordinates) ============
    public static final double LANE_1_CENTER_Y = 8.5;
    public static final double LANE_2_CENTER_Y = 24.5;
    public static final double LANE_3_CENTER_Y = 40.5;

    public static final double LANE_1_Y_MIN = 0.0,  LANE_1_Y_MAX = 16.5;
    public static final double LANE_2_Y_MIN = 16.5, LANE_2_Y_MAX = 32.5;
    public static final double LANE_3_Y_MIN = 32.5, LANE_3_Y_MAX = 48.0;

    public static final double LANE_X_MIN = 92.0;    // robot scan X
    public static final double LANE_X_MAX = 144.0;   // field boundary

    // ============ Camera intrinsics (john.yml, 1920x1080) ============
    // If VisionPortal runs at a lower resolution, scale at runtime:
    //   fx_used = FX * (actual_width  / CALIBRATION_WIDTH)
    //   fy_used = FY * (actual_height / CALIBRATION_HEIGHT)
    //   cx_used = CX * (actual_width  / CALIBRATION_WIDTH)
    //   cy_used = CY * (actual_height / CALIBRATION_HEIGHT)
    public static final int    CALIBRATION_WIDTH  = 1920;
    public static final int    CALIBRATION_HEIGHT = 1080;
    public static final double FX = 1413.5039;
    public static final double FY = 1420.5480;
    public static final double CX = 948.7182;
    public static final double CY = 514.3207;
    public static final double[] DIST_COEFFS = {
        0.07513051, -0.42239558, -0.00324586, 0.00130418, 0.68654984
    };

    // ============ Camera mount offset (robot frame at 0 deg heading) ============
    // +X_robot = forward, +Y_robot = left
    public static final double CAM_OFFSET_X = 7.98;  // 202.7mm forward of robot center
    public static final double CAM_OFFSET_Y = 5.81;  // 147.66mm left of robot center
    public static final double CAM_OFFSET_Z = 0.0;   // TODO: camera height above field (not critical for v1)
    public static final double CAM_OFFSET_HEADING_DEG = 0.0;  // camera points straight forward relative to robot

    // ============ Detection filtering (ported from Python, calibrated at 1920x1080) ============
    // Area thresholds are scaled at runtime based on actual resolution.
    public static final double MIN_CONTOUR_AREA = 3000;
    public static final double MAX_CONTOUR_AREA = 500000;
    public static final double MIN_CIRCULARITY  = 0.35;
    public static final double MAX_ASPECT_RATIO = 1.8;
    public static final double MIN_CONFIDENCE   = 0.4;
    public static final double MIN_BALL_PIXEL_DIAMETER = 5.0;

    // ============ Multi-frame merge ============
    public static final int    NUM_SCAN_FRAMES           = 3;
    public static final double MERGE_CLUSTER_RADIUS_INCHES = 6.0;
    public static final int    MIN_FRAMES_FOR_VALID      = 2;   // 2 of 3 frames
    public static final long   FRAME_CAPTURE_TIMEOUT_MS  = 1000; // max wait per frame

    // ============ Preferred VisionPortal resolution ============
    // 640x480 for best FPS on Control Hub with two blob processors.
    // Note: 4:3 aspect vs 16:9 calibration — intrinsic scaling is approximate
    // but acceptable for lane-level decisions.
    public static final int VISION_PORTAL_WIDTH  = 640;
    public static final int VISION_PORTAL_HEIGHT = 480;

    // ============ HSV fallback (tuned at venue, not used by default) ============
    // SDK's ColorRange.ARTIFACT_GREEN/PURPLE uses YCrCb and is preferred.
    // Keep these in case we need to override with a custom HSV processor.
    public static final int HSV_GREEN_H_LO = 87,  HSV_GREEN_S_LO = 78,  HSV_GREEN_V_LO = 54;
    public static final int HSV_GREEN_H_HI = 94,  HSV_GREEN_S_HI = 255, HSV_GREEN_V_HI = 255;
    public static final int HSV_PURPLE_H_LO = 123, HSV_PURPLE_S_LO = 61, HSV_PURPLE_V_LO = 96;
    public static final int HSV_PURPLE_H_HI = 143, HSV_PURPLE_S_HI = 255, HSV_PURPLE_V_HI = 255;

    // ============ Fallback ============
    public static final ChosenPath NO_DETECTION_FALLBACK = ChosenPath.PATH_1;
}
