package utility;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    private static final int BUFFER_SIZE = 5;
    private static final int REQUIRED_PRESENT = 3;

    private static final double MIN_ALPHA_NEAR = 400;
    private static final double MIN_ALPHA_FAR  = 250;

    private static final double[] GREEN_N  = {0.15, 0.55, 0.30};
    private static final double[] PURPLE_N = {0.32, 0.22, 0.46};

    private static final double GREEN_TOL  = 0.15;
    private static final double PURPLE_TOL = 0.15;

    private static final double MIN_CONFIDENCE = 0.15;

    private final ColorSensor near;
    private final ColorSensor far;

    private final SensorState nearState;
    private final SensorState farState;

    private final double[] greenProfile;
    private final double[] purpleProfile;
    private final double greenTolerance;
    private final double purpleTolerance;

    private volatile boolean backgroundMode = false;
    private volatile Result cachedResult = new Result(false, BallColor.None, 0.0);

    private volatile Result cachedNearResult = new Result(false, BallColor.None, 0.0);
    private volatile Result cachedFarResult = new Result(false, BallColor.None, 0.0);

    private DistanceSensor nearDist;
    private DistanceSensor farDist;
    private double nearDistThreshold = 50.0;
    private double farDistThreshold = 50.0;

    private int colorBurstRemaining = 0;
    private boolean distanceDetected = false;

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
        this(sensor1, sensor2, greenProfile, purpleProfile, greenTol, purpleTol,
             minAlphaNear, minAlphaFar, 50.0, 50.0);
    }

    public DualBallDetector(ColorSensor sensor1, ColorSensor sensor2,
                            double[] greenProfile, double[] purpleProfile,
                            double greenTol, double purpleTol,
                            double minAlphaNear, double minAlphaFar,
                            double nearDistThreshold, double farDistThreshold) {
        this.near = sensor1;
        this.far = sensor2;

        this.greenProfile = greenProfile;
        this.purpleProfile = purpleProfile;
        this.greenTolerance = greenTol;
        this.purpleTolerance = purpleTol;

        this.nearDistThreshold = nearDistThreshold;
        this.farDistThreshold = farDistThreshold;

        if (sensor1 instanceof DistanceSensor) {
            this.nearDist = (DistanceSensor) sensor1;
        }
        if (sensor2 instanceof DistanceSensor) {
            this.farDist = (DistanceSensor) sensor2;
        }

        nearState = new SensorState(minAlphaNear);
        farState  = new SensorState(minAlphaFar);
    }

    public void update() {
        nearState.updateFrom(near);
        farState.updateFrom(far);
    }

    public Result detectBall() {

        Result nearResult = nearState.evaluate();
        Result farResult  = farState.evaluate();

        return resolveResults(nearResult, farResult);
    }

    public Result quickCheck() {
        if (backgroundMode) {
            return cachedResult;
        }
        Result nearResult = nearState.instantRead(near);
        Result farResult  = farState.instantRead(far);

        return resolveResults(nearResult, farResult);
    }

    public void updateCacheBulkSafe() {
        Result nearResult = bulkSafeRead(near, nearState.minAlpha);
        Result farResult  = bulkSafeRead(far, farState.minAlpha);
        cachedResult = resolveResults(nearResult, farResult);
    }

    private Result bulkSafeRead(ColorSensor s, double minAlpha) {
        double a = s.alpha();
        if (a < minAlpha) {
            return new Result(false, BallColor.None, 0.0);
        }

        double r = s.red();
        double g = s.green();
        double b = s.blue();

        double sum = r + g + b;
        if (sum <= 0) {
            return new Result(false, BallColor.None, 0.0);
        }

        double rn = r / sum;
        double gn = g / sum;
        double bn = b / sum;

        double gC = confidence(rn, gn, bn, greenProfile, greenTolerance);
        double pC = confidence(rn, gn, bn, purpleProfile, purpleTolerance);

        double bestC = Math.max(gC, pC);
        if (bestC < MIN_CONFIDENCE) {
            return new Result(false, BallColor.None, bestC);
        }

        return (gC > pC)
                ? new Result(true, BallColor.Green, gC)
                : new Result(true, BallColor.Purple, pC);
    }


    public void setBackgroundMode(boolean on) {
        backgroundMode = on;
    }

    public boolean checkDistancePresent() {
        boolean nearPresent = nearDist != null
                && nearDist.getDistance(DistanceUnit.MM) < nearDistThreshold;
        boolean farPresent = farDist != null
                && farDist.getDistance(DistanceUnit.MM) < farDistThreshold;
        return nearPresent || farPresent;
    }


    public void startColorBurst(int cycles) {
        colorBurstRemaining = cycles;
    }

    public boolean isInColorBurst() {
        return colorBurstRemaining > 0;
    }

    public void colorBurstTick() {
        if (colorBurstRemaining > 0) {
            updateCacheBulkSafe();
            colorBurstRemaining--;
        }
    }

    public void setDistanceDetected(boolean detected) {
        distanceDetected = detected;
        if (!detected) {
            cachedResult = new Result(false, BallColor.None, 0.0);
            cachedNearResult = new Result(false, BallColor.None, 0.0);
            cachedFarResult = new Result(false, BallColor.None, 0.0);
        }
    }

    public boolean isDistanceDetected() {
        return distanceDetected;
    }

    private Result resolveResults(Result nearResult, Result farResult) {
        if (!nearResult.ballPresent && !farResult.ballPresent) {
            return new Result(false, BallColor.None, 0.0);
        }

        if (nearResult.ballPresent && !farResult.ballPresent) {
            return nearResult;
        }
        if (farResult.ballPresent && !nearResult.ballPresent) {
            return farResult;
        }

        if (nearResult.confidence > farResult.confidence) {
            return nearResult;
        } else {
            return farResult;
        }
    }

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
        Result instantRead(ColorSensor s) {
            NormalizedRGBA colors = ((NormalizedColorSensor) s).getNormalizedColors();

            double a = colors.alpha * 1024.0;

            if (a < minAlpha) {
                return new Result(false, BallColor.None, 0.0);
            }

            double red = colors.red;
            double grn = colors.green;
            double blu = colors.blue;

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
