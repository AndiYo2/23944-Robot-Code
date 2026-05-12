package utility;


public class SimplePoseTracker {
    private static final int MAX_SIZE = 200;
    private static final double MIN_DISTANCE_THRESHOLD = 0.5; // inches

    private final double[] xPositions = new double[MAX_SIZE];
    private final double[] yPositions = new double[MAX_SIZE];
    private int count = 0;
    private int head = 0;

    private final double[] xOutput = new double[MAX_SIZE];
    private final double[] yOutput = new double[MAX_SIZE];


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

    public double[] getXArray() {
        fillOrderedArray(xPositions, xOutput);
        return xOutput;
    }

    public double[] getYArray() {
        fillOrderedArray(yPositions, yOutput);
        return yOutput;
    }

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
