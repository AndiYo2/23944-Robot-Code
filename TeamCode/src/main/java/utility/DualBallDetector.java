package utility;

import com.qualcomm.robotcore.hardware.ColorSensor;
import Constants.EnumConstants.BallColor;

public class DualBallDetector {

    public static class Result {
        public final boolean ballPresent;
        public final BallColor color;
        public final double confidence;

        public Result(boolean present, BallColor color, double confidence) {
            this.ballPresent = present;
            this.color = color;
            this.confidence = confidence;
        }
    }

    // =============================
    // Tunables
    // =============================
    private static final int BUFFER_SIZE = 5;
    private static final int REQUIRED_PRESENT = 3;

    // Per-sensor alpha thresholds
    private static final double MIN_ALPHA_NEAR = 400;
    private static final double MIN_ALPHA_FAR  = 250;

    // Color profiles (sum-normalized, REV V2)
    private static final double[] GREEN_N  = {0.15, 0.55, 0.30};
    private static final double[] PURPLE_N = {0.32, 0.22, 0.46};

    private static final double GREEN_TOL  = 0.15;
    private static final double PURPLE_TOL = 0.15;

    private static final double MIN_CONFIDENCE = 0.15;

    // =============================
    // Hardware
    // =============================
    private final ColorSensor near;
    private final ColorSensor far;

    // =============================
    // Sensor state containers
    // =============================
    private final SensorState nearState;
    private final SensorState farState;

    // =============================
    // Instance color config (allows per-detector tuning)
    // =============================
    private final double[] greenProfile;
    private final double[] purpleProfile;
    private final double greenTolerance;
    private final double purpleTolerance;

    public DualBallDetector(ColorSensor sensor1, ColorSensor sensor2) {
        this(sensor1, sensor2, GREEN_N, PURPLE_N, GREEN_TOL, PURPLE_TOL);
    }

    public DualBallDetector(ColorSensor sensor1, ColorSensor sensor2,
                            double[] greenProfile, double[] purpleProfile,
                            double greenTol, double purpleTol) {
        this(sensor1, sensor2, greenProfile, purpleProfile, greenTol, purpleTol,
             MIN_ALPHA_NEAR, MIN_ALPHA_FAR);
    }

    public DualBallDetector(ColorSensor sensor1, ColorSensor sensor2,
                            double[] greenProfile, double[] purpleProfile,
                            double greenTol, double purpleTol,
                            double minAlphaNear, double minAlphaFar) {
        this.near = sensor1;
        this.far = sensor2;

        this.greenProfile = greenProfile;
        this.purpleProfile = purpleProfile;
        this.greenTolerance = greenTol;
        this.purpleTolerance = purpleTol;

        nearState = new SensorState(minAlphaNear);
        farState  = new SensorState(minAlphaFar);
    }

    // =============================
    // CALL EVERY LOOP
    // =============================
    public void update() {
        nearState.updateFrom(near);
        farState.updateFrom(far);
    }

    // =============================
    // SAFE TO CALL ANYTIME
    // =============================
    public Result detectBall() {

        Result nearResult = nearState.evaluate();
        Result farResult  = farState.evaluate();

        return resolveResults(nearResult, farResult);
    }

    /**
     * Single-read fallback for immediate detection.
     * Lower confidence than averaged detectBall(), but no buffer latency.
     * Use when you need instant detection and can't wait for buffer to fill.
     */
    public Result quickCheck() {
        Result nearResult = nearState.instantRead(near);
        Result farResult  = farState.instantRead(far);

        return resolveResults(nearResult, farResult);
    }

    private Result resolveResults(Result nearResult, Result farResult) {
        // If neither sees a ball
        if (!nearResult.ballPresent && !farResult.ballPresent) {
            return new Result(false, BallColor.None, 0.0);
        }

        // If only one sees a ball
        if (nearResult.ballPresent && !farResult.ballPresent) {
            return nearResult;
        }
        if (farResult.ballPresent && !nearResult.ballPresent) {
            return farResult;
        }

        // Both see a ball — choose higher confidence
        if (nearResult.confidence > farResult.confidence) {
            return nearResult;
        } else {
            return farResult;
        }
    }

    // =====================================================
    // ================= SENSOR STATE ======================
    // =====================================================
    private class SensorState {

        private final double minAlpha;

        private int index = 0;
        private int samples = 0;

        private final double[] rBuf = new double[BUFFER_SIZE];
        private final double[] gBuf = new double[BUFFER_SIZE];
        private final double[] bBuf = new double[BUFFER_SIZE];
        private final double[] aBuf = new double[BUFFER_SIZE];

        SensorState(double minAlpha) {
            this.minAlpha = minAlpha;
        }

        void updateFrom(ColorSensor s) {

            double a = s.alpha();
            aBuf[index] = a;

            if (a >= minAlpha) {
                double r = s.red();
                double g = s.green();
                double b = s.blue();

                double sum = r + g + b;
                if (sum > 0) {
                    rBuf[index] = r / sum;
                    gBuf[index] = g / sum;
                    bBuf[index] = b / sum;
                }
            }

            index = (index + 1) % BUFFER_SIZE;
            samples = Math.min(samples + 1, BUFFER_SIZE);
        }

        Result evaluate() {

            if (samples < BUFFER_SIZE) {
                return new Result(false, BallColor.None, 0.0);
            }

            int present = 0;
            double r = 0, g = 0, b = 0;

            for (int i = 0; i < BUFFER_SIZE; i++) {
                if (aBuf[i] >= minAlpha) {
                    present++;
                    r += rBuf[i];
                    g += gBuf[i];
                    b += bBuf[i];
                }
            }

            if (present < REQUIRED_PRESENT) {
                return new Result(false, BallColor.None, 0.0);
            }

            r /= present;
            g /= present;
            b /= present;

            double gC = confidence(r, g, b, greenProfile, greenTolerance);
            double pC = confidence(r, g, b, purpleProfile, purpleTolerance);

            double bestC = Math.max(gC, pC);
            if (bestC < MIN_CONFIDENCE) {
                return new Result(false, BallColor.None, bestC);
            }

            return (gC > pC)
                    ? new Result(true, BallColor.Green, gC)
                    : new Result(true, BallColor.Purple, pC);
        }

        /**
         * Single instantaneous read - bypasses buffer entirely.
         * Reads directly from hardware, applies same color matching logic.
         */
        Result instantRead(ColorSensor s) {
            double a = s.alpha();

            if (a < minAlpha) {
                return new Result(false, BallColor.None, 0.0);
            }

            double red = s.red();
            double grn = s.green();
            double blu = s.blue();

            double sum = red + grn + blu;
            if (sum <= 0) {
                return new Result(false, BallColor.None, 0.0);
            }

            double r = red / sum;
            double g = grn / sum;
            double b = blu / sum;

            double gC = confidence(r, g, b, greenProfile, greenTolerance);
            double pC = confidence(r, g, b, purpleProfile, purpleTolerance);

            double bestC = Math.max(gC, pC);
            if (bestC < MIN_CONFIDENCE) {
                return new Result(false, BallColor.None, bestC);
            }

            return (gC > pC)
                    ? new Result(true, BallColor.Green, gC)
                    : new Result(true, BallColor.Purple, pC);
        }
    }

    // =============================
    // Math helpers
    // =============================
    private double confidence(double r, double g, double b,
                              double[] ref, double tol) {
        double d = Math.sqrt(
                sq(r - ref[0]) +
                        sq(g - ref[1]) +
                        sq(b - ref[2])
        );
        return clamp(1.0 - (d / tol));
    }

    private double sq(double x) { return x * x; }

    private double clamp(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}
