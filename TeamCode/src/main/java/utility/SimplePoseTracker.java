package utility;

/**
 * Lightweight ring buffer for tracking pose history in TeleOp.
 * Stores the last MAX_SIZE (x, y) positions for field trail drawing.
 * Does NOT depend on PedroPathing PoseHistory or Follower.
 *
 * Adds a new point only when the robot has moved more than
 * MIN_DISTANCE_THRESHOLD inches from the last recorded point,
 * preventing trail clutter when stationary.
 */
public class SimplePoseTracker {
    private static final int MAX_SIZE = 200;
    private static final double MIN_DISTANCE_THRESHOLD = 0.5; // inches

    private final double[] xPositions = new double[MAX_SIZE];
    private final double[] yPositions = new double[MAX_SIZE];
    private int count = 0;
    private int head = 0;

    // Pre-allocated output arrays at max size — reused across calls
    private final double[] xOutput = new double[MAX_SIZE];
    private final double[] yOutput = new double[MAX_SIZE];

    /**
     * Record a new position. Only stores it if the robot moved
     * more than MIN_DISTANCE_THRESHOLD from the previous point.
     */
    public void addPose(double x, double y) {
        if (count > 0) {
            int lastIdx = (head - 1 + MAX_SIZE) % MAX_SIZE;
            double dx = x - xPositions[lastIdx];
            double dy = y - yPositions[lastIdx];
            if (dx * dx + dy * dy < MIN_DISTANCE_THRESHOLD * MIN_DISTANCE_THRESHOLD) {
                return;
            }
        }
        xPositions[head] = x;
        yPositions[head] = y;
        head = (head + 1) % MAX_SIZE;
        if (count < MAX_SIZE) count++;
    }

    /** Get the x positions array ordered oldest to newest (pre-allocated, do NOT store reference). */
    public double[] getXArray() {
        fillOrderedArray(xPositions, xOutput);
        return xOutput;
    }

    /** Get the y positions array ordered oldest to newest (pre-allocated, do NOT store reference). */
    public double[] getYArray() {
        fillOrderedArray(yPositions, yOutput);
        return yOutput;
    }

    /** Number of recorded positions. */
    public int getCount() {
        return count;
    }

    private void fillOrderedArray(double[] source, double[] dest) {
        if (count < MAX_SIZE) {
            System.arraycopy(source, 0, dest, 0, count);
        } else {
            int tailLength = MAX_SIZE - head;
            System.arraycopy(source, head, dest, 0, tailLength);
            System.arraycopy(source, 0, dest, tailLength, head);
        }
    }
}
