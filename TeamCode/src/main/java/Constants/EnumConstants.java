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
        IDLE,
        SCANNING_ALL_SENSORS,
        // Standard mode (sorted) - fills all 3 slots
        POSITION_TO_300,
        WAIT_POSITION,
        INTAKE_BALL_1,
        WAIT_BALL_1,
        ROTATE_TO_240,
        WAIT_ROTATION_240,
        INTAKE_BALL_2,
        WAIT_BALL_2,
        ROTATE_TO_180,
        WAIT_ROTATION_180,
        INTAKE_BALL_3,
        WAIT_BALL_3,
        // Fast mode - loads 1st ball directly into shooter
        FAST_POSITION_TO_240,
        FAST_WAIT_POSITION,
        FAST_FLIP_AND_INTAKE,
        FAST_WAIT_FLIP,
        FAST_ROTATE_TO_180,
        FAST_WAIT_ROTATION,
        FAST_INTAKE_BALL,
        FAST_WAIT_BALL,
        COMPLETE
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

    public enum ShootingMode {
        Fast,
        Sorted
    }
}
