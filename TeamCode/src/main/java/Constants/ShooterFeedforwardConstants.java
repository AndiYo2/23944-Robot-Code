package Constants;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Feedforward + PID constants for precise flywheel velocity control.
 * Uses FTCLib's SimpleMotorFeedforward combined with PIDController.
 *
 * This is SEPARATE from ShooterConstants.ShooterPIDF so you can test
 * feedforward control without affecting your current shooter.
 *
 * TUNING ORDER:
 * 1. Find kS: Slowly increase power until flywheel just starts moving
 * 2. Find kV: Run at full power, measure max velocity, kV ≈ 1.0 / maxVelocity
 * 3. Tune kI: Small increments (0.0001) until steady-state error is eliminated
 * 4. Tune kP: Small increments (0.00005) for faster recovery after shooting
 *
 * WHY FEEDFORWARD > PURE PIDF:
 * - Feedforward proactively applies power based on target velocity
 * - PID only reacts to error (slower response)
 * - Combined approach = fast spinup + precise steady-state + quick recovery
 */
@Configurable
public class ShooterFeedforwardConstants {

    // ==================== FEEDFORWARD GAINS ====================
    // These handle the "proactive" control - what power SHOULD be applied

    /**
     * Static friction compensation (kS)
     * The minimum power needed to overcome friction and start the flywheel moving.
     * HOW TO FIND: Slowly increase power from 0 until flywheel just starts spinning.
     */
    public static double kS = 0.05;

    /**
     * Velocity gain (kV)
     * Maps target velocity to motor power. Units: power per (tick/sec)
     * HOW TO FIND: Run motor at power=1.0, measure max velocity, kV = 1.0 / maxVelocity
     * Example: If max velocity = 2800 ticks/sec, kV = 1.0/2800 = 0.000357
     */
    public static double kV = 0.00035;

    /**
     * Acceleration gain (kA)
     * Compensates for flywheel inertia during spinup. Usually 0 for flywheels.
     * Only needed if you want faster spinup response.
     */
    public static double kA = 0.0;

    // ==================== PID GAINS ====================
    // These handle "reactive" control - correcting for errors

    /**
     * Proportional gain (kP)
     * Reacts to current error. Helps with faster recovery after shooting a ball.
     * TUNE LAST - after feedforward is working well.
     */
    public static double kP = 0.0001;

    /**
     * Integral gain (kI)
     * Accumulates error over time. Eliminates steady-state error.
     * TUNE SECOND - after kS and kV are set.
     * Start very small (0.0001) and increase slowly.
     */
    public static double kI = 0.0002;

    /**
     * Derivative gain (kD)
     * Reacts to rate of change of error. Usually NOT needed for velocity control.
     */
    public static double kD = 0.0;

    // ==================== TUNING PARAMETERS ====================

    /**
     * Velocity tolerance for "at target" check (ticks/sec)
     * Shooter is considered ready when within this tolerance.
     */
    public static double VELOCITY_TOLERANCE = 50.0;

    /**
     * Target velocity for tuning mode (ticks/sec)
     */
    public static double TUNING_VELOCITY = 2200.0;

    /**
     * Max velocity (ticks/sec) - measured with motor at full power
     * Used to calculate kV automatically if needed.
     */
    public static double MAX_VELOCITY = 2800.0;

    // ==================== ANTI-WINDUP ====================

    /**
     * Maximum integral accumulation (prevents windup)
     */
    public static double INTEGRAL_MAX = 0.3;
}
