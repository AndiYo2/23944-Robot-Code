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
}
