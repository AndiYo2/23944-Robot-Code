package Constants;

/**
 * All enumerations used throughout the robot code.
 */
public class EnumConstants {
    public enum AllianceColor {
        Red,
        Blue
    }

    public enum FlickState {
        Idle,
        Start,
        Extended,
        Retracted
    }

    public enum BallColor {
        Purple,
        Green,
        None
    }

    public enum IntakeState {
        Idle,
        Intaking,
        Reversing,
        StagingOnly,
        IntakeOnly,
        ReversedInBeltGo
    }

    public enum DriveState {
        Idle,
        FieldRelative,
        SlowMode,
        AutoDriving,
        Parking,
        Locked
    }

    public enum LimelightMode {
        GoalTracking,
        TagTracking
    }

    public enum ShootingMode {
        Fast,
        Sorted
    }

    public enum SensorPairState {
        UNCHECKED,
        COLOR_SCANNING,
        CONFIRMED
    }
}
