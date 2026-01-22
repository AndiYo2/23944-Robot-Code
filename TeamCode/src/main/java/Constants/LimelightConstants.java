package Constants;

import com.bylazar.configurables.annotations.Configurable;
import utility.SpindexerAndMotifStatus;

@Configurable
public class LimelightConstants {
    public static boolean isLimelightDisabled = false;

    public static SpindexerAndMotifStatus.MotifPattern motifPattern =
        new SpindexerAndMotifStatus.MotifPattern(EnumConstants.BallColor.Purple, EnumConstants.BallColor.Green, EnumConstants.BallColor.Purple);

    public static double TAG_GOAL_X = 72.0;
    public static double TAG_GOAL_Y = 143.0;

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

    public static EnumConstants.BallColor[] getMotifPatternForTag(int tagId) {
        switch (tagId) {
            case 21: return APRILTAG_21_PATTERN;
            case 22: return APRILTAG_22_PATTERN;
            case 23: return APRILTAG_23_PATTERN;
            default: return null;
        }
    }
}
