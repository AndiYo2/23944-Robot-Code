package Constants;

import com.bylazar.configurables.annotations.Configurable;
import utility.SpindexerAndMotifStatus;

@Configurable
public class LimelightConstants {
    public static final SpindexerAndMotifStatus.MotifPattern motifPattern = new SpindexerAndMotifStatus.MotifPattern();

    static {
        SpindexerAndMotifStatus.MotifPattern.setBallPattern(
            EnumConstants.BallColor.Purple, EnumConstants.BallColor.Purple, EnumConstants.BallColor.Green);
    }

    public static final EnumConstants.BallColor[] APRILTAG_21_PATTERN = {
        EnumConstants.BallColor.Green, EnumConstants.BallColor.Purple, EnumConstants.BallColor.Purple
    };
    public static final EnumConstants.BallColor[] APRILTAG_22_PATTERN = {
        EnumConstants.BallColor.Purple, EnumConstants.BallColor.Green, EnumConstants.BallColor.Purple
    };
    public static final EnumConstants.BallColor[] APRILTAG_23_PATTERN = {
        EnumConstants.BallColor.Purple, EnumConstants.BallColor.Purple, EnumConstants.BallColor.Green
    };

    public static boolean manuallySlowedForScan = false;

    // Pipeline indices
    public static final int MOTIF_PIPELINE = 5;
    public static final int LOCALIZATION_PIPELINE = 2;
    public static final int RAMP_SCAN_PIPELINE = 6;

    // Coordinate conversion constants
    public static final double METERS_TO_INCHES = 39.3701;
    public static final double FIELD_CENTER_OFFSET_INCHES = 72.0;


    public static EnumConstants.BallColor[] getMotifPatternForTag(int tagId) {
        switch (tagId) {
            case 21: return APRILTAG_21_PATTERN;
            case 22: return APRILTAG_22_PATTERN;
            case 23: return APRILTAG_23_PATTERN;
            default: return null;
        }
    }
}
