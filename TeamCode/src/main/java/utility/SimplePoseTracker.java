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

    /** Get the x positions array ordered oldest to newest. */
    public double[] getXArray() {
        return getOrderedArray(xPositions);
    }

    /** Get the y positions array ordered oldest to newest. */
    public double[] getYArray() {
        return getOrderedArray(yPositions);
    }

    /** Number of recorded positions. */
    public int getCount() {
        return count;
    }

    private double[] getOrderedArray(double[] source) {
        double[] result = new double[count];
        if (count < MAX_SIZE) {
            System.arraycopy(source, 0, result, 0, count);
        } else {
            int tailLength = MAX_SIZE - head;
            System.arraycopy(source, head, result, 0, tailLength);
            System.arraycopy(source, 0, result, tailLength, head);
        }
        return result;
    }
}
