package Constants;

import com.bylazar.configurables.annotations.Configurable;
import utility.SpindexerAndMotifStatus;

@Configurable
public class LimelightConstants {
    public static SpindexerAndMotifStatus.MotifPattern motifPattern =
        new SpindexerAndMotifStatus.MotifPattern(EnumConstants.BallColor.Purple, EnumConstants.BallColor.Purple, EnumConstants.BallColor.Green);

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

    // Coordinate conversion constants
    public static final double METERS_TO_INCHES = 39.3701;
    public static final double FIELD_CENTER_OFFSET_INCHES = 72.0;

    // Camera offset from robot center in inches (must ALSO be set in Limelight web dashboard)
    // Dashboard: http://limelight.local:5801 -> Settings -> 3D tab
    public static final double CAMERA_OFFSET_LEFT_INCHES = 2.559;   // 6.5 cm -> inches. TODO: Verify exact value
    public static final double CAMERA_OFFSET_BACK_INCHES = 5.5;     // TODO: Verify exact value
    public static final double CAMERA_OFFSET_UP_INCHES = 15.0;      // TODO: Verify exact value

    public static EnumConstants.BallColor[] getMotifPatternForTag(int tagId) {
        switch (tagId) {
            case 21: return APRILTAG_21_PATTERN;
            case 22: return APRILTAG_22_PATTERN;
            case 23: return APRILTAG_23_PATTERN;
            default: return null;
        }
    }
}
