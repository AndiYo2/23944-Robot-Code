package utility.managers;

import Constants.EnumConstants.BallColor;
import Constants.EnumConstants.ShootingMode;
import Constants.SpindexerConstants;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import subsystems.Intake;
import subsystems.Shooter;
import subsystems.Spindexer;
import utility.DualBallDetector;

import static utility.SpindexerAndMotifStatus.MotifPattern;
import static utility.SpindexerAndMotifStatus.SpindexerPattern;

/**
 * SpindexerManager - Unified manager for ball cataloging and shooting sequences.
 *
 * Uses Action-based sequences matching the AutonSequence pattern.
 *
 * Modes:
 * - Fast: Shoots nearest ball first, doesn't care about color order
 * - Sorted: Shoots balls in MotifPattern order
 */
public class SpindexerManager {
    // Subsystems
    private final Spindexer spindexer;
    private final Shooter shooter;
    private final Intake intake;

    // Sensors
    private final DualBallDetector sensorPair1;
    private final DualBallDetector sensorPair2;
    private final DualBallDetector sensorPair3;

    // Execution
    private SpindexerExecutor executor;
    private ShootingMode mode = ShootingMode.Fast;
    private String currentSequence = "Idle";

    // Ball tracking - set once during cataloging, used during shooting
    private int totalBallsInRobot = 0;   // Counted from sensors when catalog starts
    private int ballsShot = 0;           // Incremented each time we fire

    // Shooting tracking
    private int motifIndex = 0;

    // Cataloging tracking
    private BallColor ball1Color = BallColor.None;
    private BallColor ball2Color = BallColor.None;
    private BallColor ball3Color = BallColor.None;
    private boolean flippedToShooter = false;

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
        if (executor != null && executor.isRunning()) return;
        mode = (mode == ShootingMode.Fast) ? ShootingMode.Sorted : ShootingMode.Fast;
    }

    // ==================== SHOOTING SEQUENCE ====================

    /**
     * Start shooting sequence. Fires ALL balls that were counted during cataloging.
     */
    public void triggerShooting() {
        if (executor != null && executor.isRunning()) return;

        motifIndex = 2; // After preloaded (Motif[0]) and slot 1 (Motif[1]), next rotation finds Motif[2]
        ballsShot = 0;
        currentSequence = "Shooting";

        if (totalBallsInRobot == 0) {
            // No balls - just reset position
            executor = new SpindexerSequence(spindexer, shooter, intake)
                    .fire()
                .resetPosition()
                .build();
            executor.start();
            return;
        }

        executor = buildShootingSequence();
        executor.start();
    }

    private SpindexerExecutor buildShootingSequence() {
        return new SpindexerSequence(spindexer, shooter, intake)
            // Fire preloaded ball (already in shooter from cataloging)
            .fire()
            .run("Shot 1", this::incrementBallsShot)

            // Loop: flick, fire, rotate until all balls shot
            .repeatWhile(() -> ballsShot < totalBallsInRobot)
                .flick()
                .parallel(p -> p.fire().run("Rotate to next", this::rotateToNextBall).run("Shot", this::incrementBallsShot))
                .waitFor(SpindexerConstants.ROTATION_TIME)
            .endRepeat()

            // Reset position
            .resetPosition()
            .build();
    }

    private void incrementBallsShot() {
        ballsShot++;
    }

    private void rotateToNextBall() {
        if (mode == ShootingMode.Sorted) {
            if (motifIndex >= 3) return; // No more motif positions to check
            BallColor targetColor = MotifPattern.getBallColorInSlotX(motifIndex);
            spindexer.rotateToColor(targetColor);
            motifIndex++;
        } else {
            spindexer.rotateToNextClosestBall();
        }
    }

    // ==================== CATALOGING SEQUENCES ====================

    /**
     * Start cataloging sequence.
     * Counts balls from all sensors (spindexer0, transfer, ramp) to determine total.
     */
    public void triggerCataloging() {
        if (executor != null && executor.isRunning()) return;

        // Reset tracking
        ballsShot = 0;
        flippedToShooter = false;
        currentSequence = "Cataloging";

        if (mode == ShootingMode.Fast) {
            executor = buildFastCatalogSequence();
        } else {
            executor = buildSortedCatalogSequence();
        }
        executor.start();
    }

    private SpindexerExecutor buildFastCatalogSequence() {
        return new SpindexerSequence(spindexer, shooter, intake)
            // Scan sensors and initialize pattern
            .run("Scan sensors", () -> {
                scanAndAssignSlots();
                // Ball 1 is physically in slot 0
                SpindexerPattern.setBallInSlotX(0, ball1Color);
            })

            // Rotate ball 1 to shooter position (slot 0 -> slot 1)
                .rotateCCW()

                .parallel(p -> p.flick().intake())
                .run("Track ball 2", () -> SpindexerPattern.setBallInSlotX(0, ball2Color))
            // Rotate ball 2 to slot 1
            .rotateCCW()

            // Intake ball 3 into slot 0
            .intake()
            .run("Track ball 3", () -> SpindexerPattern.setBallInSlotX(0, ball3Color))
                .run("Handle 0 balls", () -> handle0Balls())
            .build();
    }

    private SpindexerExecutor buildSortedCatalogSequence() {
        return new SpindexerSequence(spindexer, shooter, intake)
            // Scan sensors and initialize
            .run("Scan sensors", () -> {
                scanAndAssignSlots();
                flippedToShooter = false;
                // Ball 1 is physically in slot 0
                SpindexerPattern.setBallInSlotX(0, ball1Color);
            })

            // Rotate ball 1 to shooter position (slot 0 -> slot 1)
            .rotateCCW()

            // In parallel: maybe flip ball 1 + intake ball 2
            .parallel(p -> p
                .possibleFlick(
                    () -> ball1Color == MotifPattern.getBallColorInSlotX(0),
                    () -> flippedToShooter = true
                )
                .intake()
                .run("Track ball 2", () -> SpindexerPattern.setBallInSlotX(0, ball2Color))
            )

            // Rotate ball 2 to slot 1
            .rotateCCW()

            // In parallel: maybe flip ball 2 + intake ball 3
            .parallel(p -> p
                .possibleFlick(
                    () -> !flippedToShooter && ball2Color == MotifPattern.getBallColorInSlotX(0),
                    () -> flippedToShooter = true
                )
                .intake()
                .run("Track ball 3", () -> SpindexerPattern.setBallInSlotX(0, ball3Color))
            )

            // Rotate ball 3 to slot 1
            .rotateCCW()

            // Maybe flip ball 3 (no more balls to intake in parallel)
            .possibleFlick(
                () -> !flippedToShooter && ball3Color == MotifPattern.getBallColorInSlotX(0),
                () -> flippedToShooter = true
            )

            // Reorder: ensure Motif[1] is in slot 1 for the next shot
            .run("Reorder for Motif[1]", () -> {
                BallColor motif1 = MotifPattern.getBallColorInSlotX(1);
                BallColor currentSlot1 = SpindexerPattern.getBallInSlotX(1);
                if (currentSlot1 != motif1 && currentSlot1 != BallColor.None) {
                    // Slot 1 has wrong color, rotate to find Motif[1]
                    spindexer.rotateToColor(motif1);
                }
            })
            .waitFor(SpindexerConstants.ROTATION_TIME)
            .build();
    }

    // ==================== HELPER METHODS ====================

    /**
     * Scans all sensors and counts total balls in robot.
     * Uses averaged detectBall() when buffer is ready, falls back to quickCheck()
     * for immediate detection when buffer hasn't filled yet.
     * Sensors: sensorPair1 = spindexer slot 0, sensorPair2 = transfer, sensorPair3 = ramp
     */
    private void scanAndAssignSlots() {
        DualBallDetector.Result r1 = detectWithFallback(sensorPair1);
        DualBallDetector.Result r2 = detectWithFallback(sensorPair2);
        DualBallDetector.Result r3 = detectWithFallback(sensorPair3);

        ball1Color = r1.ballPresent ? r1.color : BallColor.None;
        ball2Color = r2.ballPresent ? r2.color : BallColor.None;
        ball3Color = r3.ballPresent ? r3.color : BallColor.None;

        // Count total balls from all sensors
        totalBallsInRobot = 0;
        if (r1.ballPresent) totalBallsInRobot++;
        if (r2.ballPresent) totalBallsInRobot++;
        if (r3.ballPresent) totalBallsInRobot++;
    }

    /**
     * Tries averaged detection first; if buffer not ready, falls back to single-read.
     */
    private DualBallDetector.Result detectWithFallback(DualBallDetector sensor) {
        DualBallDetector.Result result = sensor.detectBall();
        if (!result.ballPresent) {
            // Buffer may not be full - try instant read
            result = sensor.quickCheck();
        }
        return result;
    }

    // ==================== UPDATE & STATUS ====================

    public void update() {
        sensorPair1.update();
        sensorPair2.update();
        sensorPair3.update();

        if (executor != null) {
            executor.update();

            if (!executor.isRunning()) {
                currentSequence = "Idle";
            }
        }
    }

    public boolean isIdle() {
        return executor == null || !executor.isRunning();
    }

    public boolean isExecuting() {
        return executor != null && executor.isRunning();
    }

    public boolean isActive() {
        return executor != null && executor.isRunning();
    }

    public String getStatus() {
        if (executor == null || !executor.isRunning()) {
            return String.format("IDLE [%s] Balls:%d Shot:%d", mode, totalBallsInRobot, ballsShot);
        }
        return String.format("%s [%s] Balls:%d Shot:%d - %s",
                currentSequence, mode, totalBallsInRobot, ballsShot, executor.getCurrentActionName());
    }

    // Getters for ball tracking
    public int getTotalBallsInRobot() { return totalBallsInRobot; }
    public int getBallsShot() { return ballsShot; }
    public int getBallsRemaining() { return totalBallsInRobot - ballsShot; }

    public String getState() {
        return (executor != null && executor.isRunning()) ? currentSequence : "IDLE";
    }

    // ==================== RESET ====================

    /**
     * Reset all ball tracking state to zero.
     * Use when balls have been manually removed or state is out of sync.
     * Will not reset during an active sequence.
     */
    public void resetBallTracking() {
        if (executor != null && executor.isRunning()) return;

        totalBallsInRobot = 0;
        ballsShot = 0;
        motifIndex = 0;
        ball1Color = BallColor.None;
        ball2Color = BallColor.None;
        ball3Color = BallColor.None;
        flippedToShooter = false;
        SpindexerPattern.clearAll();
        currentSequence = "Idle";
    }

    /**
     * Force stop any running sequence and reset all state.
     * Use for emergency reset when things are out of sync.
     */
    public void forceReset() {
        if (executor != null) {
            executor.stop();
        }
        executor = null;
        resetBallTracking();
        spindexer.resetToEmptyPosition();
    }
    public void handle0Balls(){
        if(totalBallsInRobot == 0)
            spindexer.resetToEmptyPosition();
    }
}
