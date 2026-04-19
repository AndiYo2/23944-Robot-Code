package vision;

/*
 * ======================== Example integration ========================
 * In your auton init():
 *
 *   ArtifactDetector detector = new ArtifactDetector();
 *
 *   // Build vision paths (from Paths inner class)
 *   PathChain[] goToPaths = { paths.Path1GoTo, paths.Path2GoTo, paths.Path3GoTo };
 *
 * In the command sequence (after driving to scan position):
 *
 *   ChosenPath chosen = LaneSelector.selectPath(detector, follower.getPose());
 *   PathChain goTo;
 *   switch (chosen) {
 *       case PATH_1: goTo = paths.Path1GoTo; break;
 *       case PATH_2: goTo = paths.Path2GoTo; break;
 *       case PATH_3: goTo = paths.Path3GoTo; break;
 *       default:     goTo = paths.Path1GoTo; break;
 *   }
 *   follower.followPath(goTo, true);
 *   // ... wait for completion, run intake, etc ...
 *   detector.disable();
 *
 * Or use the builder API (preferred):
 *
 *   .intakeStart()
 *   .visionCollectAndCatalog(detector, visionGoToPaths, shootPose, maxSpeed, false, .3)
 *   .shoot()
 * =====================================================================
 */

import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import Constants.EnumConstants.BallColor;

/**
 * Main entry point for vision-based lane selection.
 * Captures multiple frames, merges detections, assigns to lanes, and returns
 * the best path via a priority decision ladder.
 */
public class LaneSelector {

    private LaneSelector() {} // static utility

    /** Debug string from the last scan — read from telemetry via VisionCollectCommand.lastScanResult */
    public static String lastScanDebug = "No scan yet";

    /** Stored frames from a pre-scan at a different heading. Merged into the next selectPath call. */
    private static final List<List<FieldBall>> storedFrames = new ArrayList<>();

    /**
     * Captures frames at the current heading and stores them for the next selectPath call.
     * Call this before rotating, then call selectPath after rotating — both scans
     * are merged for better lane coverage.
     * Blocks ~500ms.
     */
    public static void captureAndStore(ArtifactDetector detector, Pose robotPose) {
        try {
            captureFrames(detector, robotPose, storedFrames);
        } catch (Exception e) {
            // Don't crash — just skip the pre-scan
        }
    }

    /**
     * Runs a full scan, merges with any stored pre-scan frames, and returns the chosen path.
     * Blocks ~500ms.
     */
    public static ChosenPath selectPath(ArtifactDetector detector, Pose robotPose) {
        try {
            // Start with any stored frames from a pre-scan
            List<List<FieldBall>> allFrames = new ArrayList<>(storedFrames);
            storedFrames.clear();

            // Capture fresh frames at current heading
            captureFrames(detector, robotPose, allFrames);

            // Merge detections across all frames and decide
            List<FieldBall> mergedBalls = clusterDetections(allFrames);

            // Count raw detections per frame for debug
            int totalRaw = 0;
            for (List<FieldBall> frame : allFrames) totalRaw += frame.size();

            ChosenPath result = decidePath(mergedBalls);
            lastScanDebug = result
                    + " | merged:" + mergedBalls.size()
                    + " | raw:" + totalRaw + "/"+allFrames.size()+"f"
                    + " | blobs G:" + detector.lastGreenBlobCount + " P:" + detector.lastPurpleBlobCount
                    + (detector.lastScanError.isEmpty() ? "" : " | ERR:" + detector.lastScanError);
            return result;
        } catch (Exception e) {
            // Don't crash the auto — fall back to default lane
            lastScanDebug = "VISION ERROR: " + e.getClass().getSimpleName() + " " + e.getMessage()
                    + " | fallback=" + VisionConstants.NO_DETECTION_FALLBACK;
            try { detector.disable(); } catch (Exception ignored) {}
            return VisionConstants.NO_DETECTION_FALLBACK;
        }
    }

    /**
     * Clusters detections across frames. A cluster is a set of field-frame
     * positions within MERGE_CLUSTER_RADIUS_INCHES of a running centroid.
     * Only the highest-confidence detection per frame per cluster is kept.
     * Clusters appearing in fewer than MIN_FRAMES_FOR_VALID frames are dropped.
     */
    private static List<FieldBall> clusterDetections(List<List<FieldBall>> allFrames) {
        List<Cluster> clusters = new ArrayList<>();

        for (int frameIdx = 0; frameIdx < allFrames.size(); frameIdx++) {
            for (FieldBall ball : allFrames.get(frameIdx)) {
                // Find nearest existing cluster
                Cluster nearest = null;
                double nearestDist = Double.MAX_VALUE;
                for (Cluster c : clusters) {
                    double dist = c.distanceTo(ball.fieldX, ball.fieldY);
                    if (dist < nearestDist) {
                        nearestDist = dist;
                        nearest = c;
                    }
                }

                if (nearest != null && nearestDist <= VisionConstants.MERGE_CLUSTER_RADIUS_INCHES) {
                    nearest.add(ball, frameIdx);
                } else {
                    Cluster newCluster = new Cluster();
                    newCluster.add(ball, frameIdx);
                    clusters.add(newCluster);
                }
            }
        }

        // Keep only clusters that appeared in enough frames
        List<FieldBall> result = new ArrayList<>();
        for (Cluster c : clusters) {
            if (c.getFrameCount() >= VisionConstants.MIN_FRAMES_FOR_VALID) {
                result.add(c.toFieldBall());
            }
        }
        return result;
    }

    /**
     * Assigns balls to lanes and applies the decision ladder.
     * A ball is in a lane if its field Y falls within the lane range
     * AND its field X falls within [LANE_X_MIN, LANE_X_MAX].
     */
    /** Debug: lane counts and ball positions from last decidePath call. */
    public static String lastLaneDebug = "";

    private static ChosenPath decidePath(List<FieldBall> balls) {
        int l1 = 0, l2 = 0, l3 = 0;
        int outOfBounds = 0;
        StringBuilder posDebug = new StringBuilder();

        for (FieldBall ball : balls) {
            posDebug.append(String.format("(%.0f,%.0f)", ball.fieldX, ball.fieldY));
            if (ball.fieldX < VisionConstants.LANE_X_MIN || ball.fieldX > VisionConstants.LANE_X_MAX) {
                outOfBounds++;
                continue;
            }

            if (ball.fieldY >= VisionConstants.LANE_1_Y_MIN && ball.fieldY < VisionConstants.LANE_1_Y_MAX) {
                l1++;
            } else if (ball.fieldY >= VisionConstants.LANE_2_Y_MIN && ball.fieldY < VisionConstants.LANE_2_Y_MAX) {
                l2++;
            } else if (ball.fieldY >= VisionConstants.LANE_3_Y_MIN && ball.fieldY < VisionConstants.LANE_3_Y_MAX) {
                l3++;
            }
        }

        lastLaneDebug = "L1:" + l1 + " L2:" + l2 + " L3:" + l3 + " OOB:" + outOfBounds + " " + posDebug;

        // Pick lane with the most balls. Ties favor L1.
        if (l1 == 0 && l2 == 0 && l3 == 0) return VisionConstants.NO_DETECTION_FALLBACK;
        if (l1 >= l2 && l1 >= l3) return ChosenPath.PATH_1;
        if (l2 > l1 && l2 >= l3) return ChosenPath.PATH_2;
        return ChosenPath.PATH_3;
    }

    /**
     * Captures NUM_SCAN_FRAMES frames and appends field-frame detections to the provided list.
     */
    private static void captureFrames(ArtifactDetector detector, Pose robotPose,
                                       List<List<FieldBall>> outFrames) {
        for (int i = 0; i < VisionConstants.NUM_SCAN_FRAMES; i++) {
            List<Detection> rawDetections = detector.scanOnce();
            if (rawDetections == null) rawDetections = new ArrayList<>();
            List<FieldBall> fieldBalls = new ArrayList<>();
            for (Detection d : rawDetections) {
                if (d != null) {
                    fieldBalls.add(BallLocalizer.toFieldFrame(d, robotPose));
                }
            }
            outFrames.add(fieldBalls);

            if (i < VisionConstants.NUM_SCAN_FRAMES - 1) {
                sleep(250);
            }
        }
    }

    private static void sleep(long ms) {
        try { Thread.sleep(ms); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }

    // ==================== Cluster helper ====================

    private static class Cluster {
        // Best detection per frame (keyed by frame index)
        private final Map<Integer, FieldBall> bestPerFrame = new HashMap<>();

        void add(FieldBall ball, int frameIndex) {
            FieldBall existing = bestPerFrame.get(frameIndex);
            if (existing == null || ball.confidence > existing.confidence) {
                bestPerFrame.put(frameIndex, ball);
            }
        }

        double getCentroidX() {
            double sum = 0;
            for (FieldBall b : bestPerFrame.values()) sum += b.fieldX;
            return sum / bestPerFrame.size();
        }

        double getCentroidY() {
            double sum = 0;
            for (FieldBall b : bestPerFrame.values()) sum += b.fieldY;
            return sum / bestPerFrame.size();
        }

        double distanceTo(double x, double y) {
            double dx = getCentroidX() - x;
            double dy = getCentroidY() - y;
            return Math.sqrt(dx * dx + dy * dy);
        }

        int getFrameCount() {
            return bestPerFrame.size();
        }

        FieldBall toFieldBall() {
            double sumConf = 0;
            for (FieldBall b : bestPerFrame.values()) sumConf += b.confidence;
            // Use the color from the first detection in the cluster
            BallColor majorityColor = bestPerFrame.values().iterator().next().color;
            return new FieldBall(
                    getCentroidX(), getCentroidY(),
                    majorityColor,
                    sumConf / bestPerFrame.size(),
                    bestPerFrame.size()
            );
        }
    }
}
