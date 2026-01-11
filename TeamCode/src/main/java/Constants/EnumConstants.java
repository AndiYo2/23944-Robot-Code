package Constants;

/**
 * All enumerations used throughout the robot code.
 */
public class EnumConstants {
    public enum FieldState {
        IdleZone,
        ShootingZone,
        ParkZone,
        PenaltyZone
    }

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

    public enum RotationState {
        IDLE,      // Ready for commands
        ROTATING   // Busy rotating
    }

    public enum ShooterCases {
        Idle,
        Start,
        SpindexerFlicking,
        ShooterFlicking,
        SpindexerRotating,
        BallShot
    }

    public enum BallColor {
        Purple,
        Green,
        None
    }

    public enum CatalogingCases {
        Idle,
        Scanning,
        WaitingForRotation,
        RetryRotation,
        RotateToEndLocation
    }

    public enum IntakeState {
        Idle,
        Intaking,
        Reversing,
        StagingOnly,
        IntakeOnly
    }

    public enum DriveState {
        Idle,
        FieldRelative,
        RobotRelative,
        SlowMode,
        AutoDriving,
        Parking,
        Locked
    }

    public enum LimelightMode {
        GoalTracking,
        TagTracking
    }
}
