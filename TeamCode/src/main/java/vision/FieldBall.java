package vision;

import Constants.EnumConstants.BallColor;

/**
 * A ball detection transformed to field-frame coordinates.
 * After multi-frame merge, confidence is averaged and frameCount
 * indicates how many of the scan frames contained this ball.
 */
public class FieldBall {
    public final double fieldX;      // inches, field frame
    public final double fieldY;      // inches, field frame
    public final BallColor color;
    public final double confidence;  // averaged across merged frames
    public final int frameCount;     // how many of NUM_SCAN_FRAMES saw this ball

    public FieldBall(double fieldX, double fieldY, BallColor color,
                     double confidence, int frameCount) {
        this.fieldX = fieldX;
        this.fieldY = fieldY;
        this.color = color;
        this.confidence = confidence;
        this.frameCount = frameCount;
    }
}
