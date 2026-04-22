package vision;

/**
 * Constants for the vision-based ball detection and corridor planner.
 *
 * Camera intrinsics are the SDK's built-in Robert-Atkinson C920 calibration at
 * the actual 640x480 runtime resolution — no cross-aspect-ratio scaling.
 *
 * Runtime-tunable fields (non-final) are adjusted via VisionTuningTeleOp and
 * persisted to /sdcard/FIRST/vision_tuning.json.
 */
public class VisionConstants {
    // ============ Ball physical ============
    public static final double BALL_DIAMETER_INCHES = 5.0;
    public static final double INTAKE_WIDTH_INCHES  = 16.0;

    // ============ Camera intrinsics (SDK built-in C920 @ 640x480) ============
    // Source: FTC SDK teamwebcamcalibrations.xml (Robert Atkinson / 3DF Zephyr).
    // Calibrated AT the runtime resolution — scaleX/scaleY in ArtifactDetector
    // become 1.0 (no-op), which is correct.
    public static final int    CALIBRATION_WIDTH  = 640;
    public static final int    CALIBRATION_HEIGHT = 480;
    public static final double FX = 622.001;
    public static final double FY = 622.001;
    public static final double CX = 319.803;
    public static final double CY = 241.251;
    public static final double[] DIST_COEFFS = {
        0.1208, -0.2616, 0.0, 0.0, 0.103
    };

    // ============ Camera mount offset (robot frame at 0 deg heading) ============
    // +X_robot = forward, +Y_robot = left
    public static final double CAM_OFFSET_X = 7.98;   // 202.7mm forward of robot center
    public static final double CAM_OFFSET_Y = 5.81;   // 147.66mm left of robot center
    public static final double CAM_OFFSET_Z = 3.0;    // camera lens-center height above field (inches)
    public static final double CAM_OFFSET_HEADING_DEG = 0.0;

    // Camera downward tilt from horizontal, in degrees. The current distance
    // model (horizontal pixel width) does NOT use this value — pitch only
    // affects WHERE the ball appears in the image, not the silhouette width.
    // Exposed as a tunable for future ground-plane-projection work and for
    // documentation of the physical mount. Measure with a digital level.
    public static double CAM_PITCH_DEG = 20.0;

    // ============ Detection filtering (640x480 native) ============
    // Area thresholds scaled down from old 1920x1080 values by pixel-count ratio (÷ 9).
    public static final double MIN_CONTOUR_AREA        = 333;    // ≈ 3000 / 9
    public static final double MAX_CONTOUR_AREA        = 55555;  // ≈ 500000 / 9
    public static final double MIN_CIRCULARITY         = 0.35;
    public static final double MAX_ASPECT_RATIO        = 1.8;
    public static final double MIN_CONFIDENCE          = 0.4;
    public static final double MIN_BALL_PIXEL_DIAMETER = 5.0;

    // ============ Multi-frame merge ============
    public static final int    NUM_SCAN_FRAMES             = 3;
    public static final double MERGE_CLUSTER_RADIUS_INCHES = 6.0;
    public static final int    MIN_FRAMES_FOR_VALID        = 2;    // 2 of 3 frames
    public static final long   FRAME_CAPTURE_TIMEOUT_MS    = 1000; // max wait per frame

    // ============ VisionPortal resolution ============
    public static final int VISION_PORTAL_WIDTH  = 640;
    public static final int VISION_PORTAL_HEIGHT = 480;

    // ============ Camera controls (runtime-tunable via VisionTuningTeleOp) ============
    // Baselines from FIRST's ConceptAprilTagOptimizeExposure sample: 5ms exposure,
    // max gain. WB is folk-knowledge default for mixed gym LED+fluorescent.
    public static long EXPOSURE_MS = 5L;   // [0..204]
    public static int  GAIN        = 255;  // [0..255]
    public static int  WB_KELVIN   = 4500; // [2000..6500]

    // ============ Corridor planner ============
    public static double CORRIDOR_HALF_WIDTH_IN     = 8.0;   // 16" intake / 2
    public static double CORRIDOR_MAX_TRAVEL_IN     = 40.0;  // cap forward sweep
    public static double CORRIDOR_HEADING_RANGE_DEG = 45.0;  // ± from current heading
    public static double CORRIDOR_HEADING_STEP_DEG  = 1.0;   // sweep resolution
    public static int    CORRIDOR_MAX_BALLS         = 3;
    public static double CORRIDOR_K_TURN            = 0.5;   // per 90° turn
    public static double CORRIDOR_K_LENGTH          = 0.005; // per inch travel (lowered: old 0.02 auto-rejected single balls past ~30")
    public static double CORRIDOR_MIN_SCORE         = 0.3;   // below → no-plan (lowered from 0.5 so a single confident ball at range still plans)
    public static double CORRIDOR_APPROACH_EXTRA_IN = 4.0;   // path endpoint: last ball + this
    public static double CORRIDOR_FINISH_BUFFER_IN  = 2.0;   // geometric cut-short: end when robot center passes last ball by this many inches

    // ============ Panels camera preview (AUTO only; TeleOp calls stopVisionPortal) ============
    // 15 fps costs ~3-5ms/frame on the camera thread (NOT scheduler loop).
    // 0 disables the Panels stream entirely.
    public static int PANELS_STREAM_FPS = 15;
}
