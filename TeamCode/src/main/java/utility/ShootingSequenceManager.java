package utility;

import subsystems.Spindexer;
import subsystems.Shooter;
import utility.RobotConstants.Enums.FlickState;
import utility.RobotConstants.Enums.ShooterCases;

/**
 * Manages the shooting sequence state machine and coordinates
 * between Spindexer and Shooter subsystems.
 */
public class ShootingSequenceManager {
    private final Spindexer spindexer;
    private final Shooter shooter;
    private final RobotHardware robot;

    private ShooterCases currentState = ShooterCases.Idle;

    public ShootingSequenceManager(Spindexer spindexer, Shooter shooter, RobotHardware robot) {
        this.spindexer = spindexer;
        this.shooter = shooter;
        this.robot = robot;
    }

    /**
     * Start a new shooting sequence using the current mode (SMART or DUMP_ONLY)
     */
    public void startShootingSequence() {
        if (isIdle()) {
            RobotConstants.MotiffPattern goalPattern = RobotConstants.Limelight.motiffPatern;
            spindexer.startShootingSequence(goalPattern);
            processNextAction();
        }
    }

    /**
     * Toggle between SMART and DUMP_ONLY shooting modes
     */
    public void toggleShootingMode() {
        if (ShootingStrategy.getStrategyMode() == ShootingStrategy.StrategyMode.SMART) {
            ShootingStrategy.setStrategyMode(ShootingStrategy.StrategyMode.FAST);
        } else {
            ShootingStrategy.setStrategyMode(ShootingStrategy.StrategyMode.SMART);
        }
    }

    /**
     * Update the shooting state machine - call this every loop
     */
    public void update() {
        switch (currentState) {
            case Idle:
                return;
            case Start:
                handleStart();
                break;
            case SpindexerFlicking:
                handleSpindexerFlicking();
                break;
            case ShooterFlicking:
                handleShooterFlicking();
                break;
            case BallShot:
                handleBallShot();
                break;
        }
    }

    public boolean isExecuting() {
        return spindexer.hasMoreActions();
    }

    public boolean isIdle() {
        return currentState == ShooterCases.Idle;
    }

    // ============= Private State Handlers =============

    private void handleStart() {
        spindexer.flickBallOut();
        currentState = ShooterCases.SpindexerFlicking;
    }

    private void handleSpindexerFlicking() {
        if (spindexer.getFlipperState() == FlickState.Extended) {
            shooter.shootBall();
            currentState = ShooterCases.ShooterFlicking;
        }
    }

    private void handleShooterFlicking() {
        if (spindexer.getFlipperState() == FlickState.Retracted) {
            currentState = ShooterCases.BallShot;
        }
    }

    private void handleBallShot() {
        robot.spindexerPattern.setBallPatternNone(1); // Clear shooter position
        spindexer.completeCurrentAction();
        currentState = ShooterCases.Idle;

        if (spindexer.hasMoreActions()) {
            processNextAction();
        }
    }

    private void processNextAction() {
        if (isIdle() && spindexer.hasMoreActions()) {
            ShootingStrategy.Action nextAction = spindexer.getNextAction();

            switch (nextAction) {
                case SHOOT:
                    currentState = ShooterCases.Start;
                    break;
                case ROTATE_FORWARD:
                    executeRotation(120);
                    break;
                case ROTATE_BACKWARD:
                    executeRotation(-120);
                    break;
            }
        }
    }

    private void executeRotation(double degrees) {
        spindexer.rotate(degrees);
        spindexer.completeCurrentAction();
        processNextAction(); // Immediately process the next action
    }
}