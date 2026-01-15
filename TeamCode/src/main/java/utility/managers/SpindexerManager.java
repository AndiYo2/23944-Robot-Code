package utility.managers;

import Constants.EnumConstants.BallColor;
import Constants.EnumConstants.ShootingMode;
import Constants.SpindexerConstants;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import subsystems.Intake;
import subsystems.Shooter;
import subsystems.Spindexer;
import utility.DualBallDetector;

import static utility.managers.Step.*;
import static utility.SpindexerAndMotifStatus.MotifPattern;
import static utility.SpindexerAndMotifStatus.SpindexerPattern;

/**
 * SpindexerManager - Unified manager for ball cataloging and shooting sequences.
 *
 * Uses step-based sequences for clean, readable, and editable state machine logic.
 *
 * Modes:
 * - Fast: Shoots nearest ball first, doesn't care about color order
 * - Sorted: Shoots balls in MotifPattern order
 *
 * Shooting: Right trigger fires ALL balls in sequence (loops until empty)
 * Cataloging: Y button triggers ball intake and pre-loading
 */
public class SpindexerManager {
    // Subsystems
    private final Spindexer spindexer;
    private final Shooter shooter;
    private final Intake intake;
    private final Telemetry telemetry;

    // Sensors
    private final DualBallDetector sensorPair1;  // At spindexer intake slot
    private final DualBallDetector sensorPair2;  // Middle belt
    private final DualBallDetector sensorPair3;  // Furthest out

    // Execution
    private final StepExecutor executor = new StepExecutor();
    private ShootingMode mode = ShootingMode.Fast;
    private String currentSequence = "Idle";

    // Shooting tracking
    private int ballsShot = 0;
    private int motifIndex = 0;  // For sorted mode: which motif position we're shooting

    // Cataloging tracking
    private BallColor ball1Color = BallColor.None;
    private BallColor ball2Color = BallColor.None;
    private BallColor ball3Color = BallColor.None;
    private boolean flippedToShooter = false;  // Track if we flipped a ball to shooter (sorted mode)

    public SpindexerManager(
            Spindexer spindexer,
            Shooter shooter,
            Intake intake,
            Telemetry telemetry,
            DualBallDetector sensorPair1,
            DualBallDetector sensorPair2,
            DualBallDetector sensorPair3
    ) {
        this.spindexer = spindexer;
        this.shooter = shooter;
        this.intake = intake;
        this.telemetry = telemetry;
        this.sensorPair1 = sensorPair1;
        this.sensorPair2 = sensorPair2;
        this.sensorPair3 = sensorPair3;
    }

    // ==================== MODE CONTROL ====================

    public void setMode(ShootingMode mode) {
        this.mode = mode;
    }

    public ShootingMode getMode() {
        return mode;
    }

    public void toggleMode() {
        if (executor.isRunning()) return;
        mode = (mode == ShootingMode.Fast) ? ShootingMode.Sorted : ShootingMode.Fast;
    }

    // ==================== SHOOTING SEQUENCE ====================

    /**
     * Start shooting sequence. Fires ALL balls until empty.
     * Works with any number of balls (0-3).
     */
    public void triggerShooting() {
        if (executor.isRunning()) return;

        ballsShot = 0;
        motifIndex = 0;
        currentSequence = "Shooting";

        // Check if there's even a ball to shoot
        if (SpindexerPattern.getBallCount() == 0) {
            // No balls - just reset position
            executor.start(new Step[] {
                action("Reset position", () -> spindexer.setDegree(SpindexerConstants.EMPTY_RESET_DEGREES)),
                done()
            });
            return;
        }

        executor.start(buildShootingSequence());
    }

    private Step[] buildShootingSequence() {
        return new Step[] {
            // [0] Fire preloaded ball in shooter
            action("Fire shooter", () -> shooter.triggerShot()),
            // [1] Wait for shot to complete
            waitFor(SpindexerConstants.SHOOTER_FLIPPER_TIME),

            // [2] Check if more balls in spindexer - if none, skip to reset
            ifThen("Check for more balls", () -> SpindexerPattern.getBallCount() == 0, 9),

            // [3] LOOP START: Load and fire remaining balls
            action("Flip next ball", () -> spindexer.triggerFlick()),
            // [4] Wait for flip
            waitFor(SpindexerConstants.SPINDEXER_FLIPPER_OUT_TIME),
            // [5] Fire loaded ball
            action("Fire loaded", () -> shooter.triggerShot()),
            // [6] Wait for shot
            waitFor(SpindexerConstants.SHOOTER_FLIPPER_TIME),

            // [7] Check if more balls - if none, exit loop to reset
            ifThen("Check for more balls", () -> SpindexerPattern.getBallCount() == 0, 4),

            // [8] Rotate to next ball (mode determines nearest vs color order)
            action("Rotate to next", this::rotateToNextBall),
            // [9] Wait for rotation
            waitFor(SpindexerConstants.ROTATION_TIME),
            // [10] Loop back to [3] if balls remain
            loopWhile(() -> SpindexerPattern.getBallCount() > 0, 7),

            // [11] Reset spindexer position
            action("Reset position", () -> spindexer.setDegree(SpindexerConstants.EMPTY_RESET_DEGREES)),
            // [12] Done
            done()
        };
    }

    private void rotateToNextBall() {
        if (mode == ShootingMode.Sorted) {
            BallColor targetColor = MotifPattern.getBallColorInSlotX(motifIndex);
            spindexer.rotateToColor(targetColor);
            motifIndex++;
        } else {
            spindexer.rotateToNextClosestBall();
        }
    }

    // ==================== FAST CATALOGING SEQUENCE ====================

    /**
     * Start cataloging sequence.
     * Fast mode: Loads first ball directly to shooter, rest to spindexer.
     * Sorted mode: Only preloads to shooter if ball matches MotifPattern[0].
     */
    public void triggerCataloging() {
        if (executor.isRunning()) return;

        flippedToShooter = false;
        currentSequence = "Cataloging";

        if (mode == ShootingMode.Fast) {
            executor.start(buildFastCatalogSequence());
        } else {
            executor.start(buildSortedCatalogSequence());
        }
    }

    private Step[] buildFastCatalogSequence() {
        return new Step[] {
            // [0] Scan sensors and store ball colors
            action("Scan sensors", this::scanAndAssignSlots),

            // [1] Rotate to 240 degrees
            action("Rotate 240", () -> spindexer.setDegree(240)),
            // [2] Wait for rotation
            waitFor(SpindexerConstants.ROTATION_TIME),

            // [3] Flip to position for shooter loading
            action("Flip to shooter", () -> spindexer.triggerFlick()),
            // [4] Wait for flip
            waitFor(SpindexerConstants.SPINDEXER_FLIPPER_OUT_TIME),

            // [5] Run intake - ball 1 goes to shooter
            action("Run intake (ball 1)", () -> intake.runIntake()),
            // [6] Wait for intake
            waitFor(SpindexerConstants.INTAKE_TIMING),
            // [7] Stop intake
            action("Stop intake", () -> intake.stopIntake()),

            // [8] Rotate to 180 for second ball
            action("Rotate 180", () -> spindexer.setDegree(180)),
            // [9] Wait for rotation
            waitFor(SpindexerConstants.ROTATION_TIME),

            // [10] Run intake - ball 2 goes to slot 0
            action("Run intake (ball 2)", () -> intake.runIntake()),
            // [11] Wait for intake
            waitFor(SpindexerConstants.INTAKE_TIMING),
            // [12] Stop intake and assign ball to slot
            action("Stop & assign", () -> {
                intake.stopIntake();
                SpindexerPattern.setBallInSlotX(0, ball2Color);
            }),

            // [13] Done
            done()
        };
    }

    // ==================== SORTED CATALOGING SEQUENCE ====================

    private Step[] buildSortedCatalogSequence() {
        return new Step[] {
            // [0] Scan sensors and store ball colors
            action("Scan sensors", () -> {
                scanAndAssignSlots();
                flippedToShooter = false;
            }),

            // [1] Rotate to 240 degrees
            action("Rotate 240", () -> spindexer.setDegree(240)),
            // [2] Wait for rotation
            waitFor(SpindexerConstants.ROTATION_TIME),

            // [3] Check if ball1 matches MotifPattern[0] - if so, flip
            ifThen("Ball1 matches?", () -> {
                BallColor expected = MotifPattern.getBallColorInSlotX(0);
                return ball1Color == expected;
            }, 0),  // Don't skip, just check and flip conditionally

            // [4] Conditional flip (only if ball1 matches)
            action("Maybe flip", () -> {
                BallColor expected = MotifPattern.getBallColorInSlotX(0);
                if (ball1Color == expected) {
                    spindexer.triggerFlick();
                    flippedToShooter = true;
                }
            }),
            // [5] Wait for possible flip
            waitFor(SpindexerConstants.SPINDEXER_FLIPPER_OUT_TIME),

            // [6] Run intake
            action("Run intake (ball 1)", () -> intake.runIntake()),
            // [7] Wait for intake
            waitFor(SpindexerConstants.INTAKE_TIMING),
            // [8] Stop intake and track ball
            action("Stop & track", () -> {
                intake.stopIntake();
                if (!flippedToShooter) {
                    // Ball1 goes to slot 1 (at 240 degrees)
                    SpindexerPattern.setBallInSlotX(1, ball1Color);
                }
            }),

            // [9] Rotate to 180 for second ball
            action("Rotate 180", () -> spindexer.setDegree(180)),
            // [10] Wait for rotation
            waitFor(SpindexerConstants.ROTATION_TIME),

            // [11] Check if ball2 matches MotifPattern[0] AND we haven't flipped yet
            action("Maybe flip ball2", () -> {
                BallColor expected = MotifPattern.getBallColorInSlotX(0);
                if (!flippedToShooter && ball2Color == expected) {
                    spindexer.triggerFlick();
                    flippedToShooter = true;
                }
            }),
            // [12] Wait for possible flip
            waitFor(SpindexerConstants.SPINDEXER_FLIPPER_OUT_TIME),

            // [13] Run intake
            action("Run intake (ball 2)", () -> intake.runIntake()),
            // [14] Wait for intake
            waitFor(SpindexerConstants.INTAKE_TIMING),
            // [15] Stop intake and assign
            action("Stop & assign", () -> {
                intake.stopIntake();
                SpindexerPattern.setBallInSlotX(0, ball2Color);
            }),

            // [16] Check if we need to flip ball3 to shooter (if nothing flipped yet)
            ifThen("Need ball3 flip?", () -> {
                BallColor expected = MotifPattern.getBallColorInSlotX(0);
                return !flippedToShooter && ball3Color == expected;
            }, 0),  // Check but don't skip

            // [17] Final flip if needed
            action("Maybe flip ball3", () -> {
                BallColor expected = MotifPattern.getBallColorInSlotX(0);
                if (!flippedToShooter && ball3Color == expected) {
                    spindexer.triggerFlick();
                    flippedToShooter = true;
                }
            }),
            // [18] Wait for possible flip
            waitFor(SpindexerConstants.SPINDEXER_FLIPPER_OUT_TIME),

            // [19] Done
            done()
        };
    }

    // ==================== HELPER METHODS ====================

    /**
     * Scan all 3 sensor pairs and store colors for preemptive assignment.
     */
    private void scanAndAssignSlots() {
        DualBallDetector.Result r1 = sensorPair1.detectBall();
        DualBallDetector.Result r2 = sensorPair2.detectBall();
        DualBallDetector.Result r3 = sensorPair3.detectBall();

        ball1Color = r1.ballPresent ? r1.color : BallColor.None;
        ball2Color = r2.ballPresent ? r2.color : BallColor.None;
        ball3Color = r3.ballPresent ? r3.color : BallColor.None;
    }

    // ==================== UPDATE & STATUS ====================

    public void update() {
        // Always update sensors
        sensorPair1.update();
        sensorPair2.update();
        sensorPair3.update();

        // Update step executor
        executor.update();

        // Reset sequence name when done
        if (!executor.isRunning()) {
            currentSequence = "Idle";
        }
    }

    public boolean isIdle() {
        return !executor.isRunning();
    }

    public boolean isExecuting() {
        return executor.isRunning();
    }

    public boolean isActive() {
        return executor.isRunning();
    }

    public String getStatus() {
        if (!executor.isRunning()) {
            return String.format("IDLE [%s]", mode);
        }
        return String.format("%s [%s] - %s",
                currentSequence, mode, executor.getCurrentStepName());
    }

    /**
     * For compatibility with existing code that checks state.
     * Returns a simple string representation of current state.
     */
    public String getState() {
        return executor.isRunning() ? currentSequence : "IDLE";
    }
}
