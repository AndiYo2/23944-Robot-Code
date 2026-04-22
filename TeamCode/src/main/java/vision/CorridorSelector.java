package vision;

import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import Constants.EnumConstants.BallColor;

/**
 * Main entry point for vision-based corridor selection.
 *
 * Pipeline: capture NUM_SCAN_FRAMES → cluster across frames → run
 * CorridorPlanner → return a Sweep (heading + endPose + captured balls + score).
 *
 * Callers check {@code sweep.isEmpty()} and {@code sweep.score >=
 * VisionConstants.CORRIDOR_MIN_SCORE} to decide whether to drive.
 *
 * Failure mode: on exception, returns {@link CorridorPlanner.Sweep#EMPTY} and
 * logs loudly to {@link #lastScanDebug}. No silent fallback.
 */
public class CorridorSelector {

    private CorridorSelector() {} // static utility

    /** Debug string from the last scan — surfaced via VisionCollectCommand.lastScanResult. */
    public static String lastScanDebug = "No scan yet";

    /** How many raw detections were merged into the last Sweep (for telemetry). */
    public static int lastMergedBallCount = 0;

    /** Stored frames from a pre-scan at a different heading. Merged into the next selectCorridor call. */
    private static final List<List<FieldBall>> storedFrames = new ArrayList<>();

    /**
     * Captures frames at the current heading and stores them for the next
     * selectCorridor call. Call before rotating, then call selectCorridor
     * after rotating — both scans are merged for better ball coverage.
     * Blocks ~500ms.
     */
    public static void captureAndStore(ArtifactDetector detector, Pose robotPose) {
        try {
            captureFrames(detector, robotPose, storedFrames);
        } catch (Exception e) {
            // Don't crash — skip the pre-scan.
        }
    }

    /**
     * Full scan: capture → cluster → corridor plan. Merges any stored pre-scan
     * frames. Blocks ~500ms. Never throws; returns {@link CorridorPlanner.Sweep#EMPTY}
     * on any error (and logs via {@link #lastScanDebug}).
     */
    public static CorridorPlanner.Sweep selectCorridor(ArtifactDetector detector, Pose robotPose) {
        try {
            List<List<FieldBall>> allFrames = new ArrayList<>(storedFrames);
            storedFrames.clear();

            captureFrames(detector, robotPose, allFrames);

            List<FieldBall> mergedBalls = clusterDetections(allFrames);
            lastMergedBallCount = mergedBalls.size();

            int totalRaw = 0;
            for (List<FieldBall> frame : allFrames) totalRaw += frame.size();

            CorridorPlanner.Sweep sweep = CorridorPlanner.plan(robotPose, mergedBalls);

            lastScanDebug = (sweep.isEmpty() ? "EMPTY" : String.format("hdg=%.0f° cap=%d score=%.2f",
                    Math.toDegrees(sweep.headingRad), sweep.captured.size(), sweep.score))
                    + " | merged:" + mergedBalls.size()
                    + " | raw:" + totalRaw + "/" + allFrames.size() + "f"
                    + " | blobs G:" + detector.lastGreenBlobCount + " P:" + detector.lastPurpleBlobCount
                    + (detector.lastScanError.isEmpty() ? "" : " | ERR:" + detector.lastScanError);
            return sweep;
        } catch (Exception e) {
            lastScanDebug = "VISION ERROR: " + e.getClass().getSimpleName() + " " + e.getMessage();
            android.util.Log.e("CorridorSelector", "selectCorridor failed", e);
            return CorridorPlanner.Sweep.EMPTY;
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

        List<FieldBall> result = new ArrayList<>();
        for (Cluster c : clusters) {
            if (c.getFrameCount() >= VisionConstants.MIN_FRAMES_FOR_VALID) {
                result.add(c.toFieldBall());
            }
        }
        return result;
    }

    /** Captures NUM_SCAN_FRAMES frames and appends field-frame detections to outFrames. */
    private static void captureFrames(ArtifactDetector detector, Pose robotPose,
                                       List<List<FieldBall>> outFrames) {
        for (int i = 0; i < VisionConstants.NUM_SCAN_FRAMES; i++) {
            List<Detection> rawDetections = detector.scanOnce();
            if (rawDetections == null) rawDetections = new ArrayList<>();
            List<FieldBall> fieldBalls = new ArrayList<>();
            for (Detection d : rawDetections) {
                if (d != null) fieldBalls.add(BallLocalizer.toFieldFrame(d, robotPose));
            }
            outFrames.add(fieldBalls);
            if (i < VisionConstants.NUM_SCAN_FRAMES - 1) sleep(250);
        }
    }

    private static void sleep(long ms) {
        try { Thread.sleep(ms); }
        catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }

    // ==================== Cluster helper ====================

    private static class Cluster {
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
