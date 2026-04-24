package utility;

import Constants.EnumConstants;

/**
 * Static state containers for spindexer and motif ball patterns.
 *
 * DESIGN NOTE: These classes use static state intentionally to allow data sharing
 * between Autonomous and TeleOp modes. The FTC SDK resets objects between OpModes,
 * but static state persists, enabling autonomous routines to set the ball pattern
 * which TeleOp can then read and continue from.
 *
 * All methods are static to provide a consistent API for accessing shared state.
 */
public class SpindexerAndMotifStatus {

    /**
     * Tracks the current ball pattern in the spindexer.
     *
     * Slot positions:
     * - Slot 0: Intake position
     * - Slot 1: Shooter/Outtake position
     * - Slot 2: Top storage position
     *
     * All methods are static - use the class directly (SpindexerPattern.getBallInSlotX(0))
     */
    public static class SpindexerPattern {
        private static EnumConstants.BallColor[] spindexerPattern = {
            EnumConstants.BallColor.None,
            EnumConstants.BallColor.None,
            EnumConstants.BallColor.None
        };

        /** Gets the ball color in the specified slot (0-2) */
        public static synchronized EnumConstants.BallColor getBallInSlotX(int x) {
            return spindexerPattern[x];
        }

        /** Sets the ball color in the specified slot */
        public static synchronized void setBallInSlotX(int x, EnumConstants.BallColor ballType) {
            spindexerPattern[x] = ballType;
        }

        /** Sets all three slot colors at once */
        public static synchronized void setBallPattern(EnumConstants.BallColor zero, EnumConstants.BallColor one, EnumConstants.BallColor two) {
            spindexerPattern = new EnumConstants.BallColor[]{zero, one, two};
        }

        /** Clears all slots (sets all to None) */
        public static synchronized void clearAll() {
            setBallPattern(EnumConstants.BallColor.None, EnumConstants.BallColor.None, EnumConstants.BallColor.None);
        }

        /** Rotates ball pattern counter-clockwise: 0->1->2->0 */
        public static synchronized void rotateBallsCCW() {
            EnumConstants.BallColor temp0 = spindexerPattern[0];
            spindexerPattern[0] = spindexerPattern[2];
            spindexerPattern[2] = spindexerPattern[1];
            spindexerPattern[1] = temp0;
        }

        /** Rotates ball pattern clockwise: 0<-1<-2<-0 */
        public static synchronized void rotateBallsCW() {
            EnumConstants.BallColor temp0 = spindexerPattern[0];
            spindexerPattern[0] = spindexerPattern[1];
            spindexerPattern[1] = spindexerPattern[2];
            spindexerPattern[2] = temp0;
        }

        /** Returns formatted string showing pattern: [I:color S:color T:color] */
        public static String getSpindexerPatternString() {
            return String.format("[I:%s S:%s T:%s]",
                    getBallInSlotX(0),  // I = Intake
                    getBallInSlotX(1),  // S = Shooter
                    getBallInSlotX(2)); // T = Top storage
        }

        /** Returns the number of slots containing balls (0-3) */
        public static int getBallCount() {
            int count = 0;
            for (int i = 0; i < 3; i++) {
                if (spindexerPattern[i] != EnumConstants.BallColor.None) {
                    count++;
                }
            }
            return count;
        }
    }

    /**
     * Tracks the number of balls deposited into the classifier ramp.
     * Used by scan-based sorted shooting to determine the correct motif-shifted
     * shooting order. Maintains a manual counter that can be corrected by
     * Limelight ramp scans.
     *
     * All methods are static and synchronized for cross-OpMode persistence.
     */
    public static class RampTracker {
        private static int ballsInRamp = 0;
        private static EnumConstants.BallColor[] originalMotif = null;
        public static String lastShiftDebug = "";

        /** Gets the current ball count in the ramp. */
        public static synchronized int getBallsInRamp() { return ballsInRamp; }

        /** Sets the ball count (used by Limelight scan correction). */
        public static synchronized void setBallsInRamp(int count) { ballsInRamp = count; }

        /** Increments the counter after shooting. */
        public static synchronized void addShotBalls(int count) { ballsInRamp += count; }

        /** Resets the counter and saved original motif. */
        public static synchronized void clear() {
            ballsInRamp = 0;
            originalMotif = null;
        }

        /**
         * Shifts the motif pattern based on current ramp count.
         * On first call, saves the current MotifPattern as the original
         * (should be called after limelightScan() has detected the tag).
         * Sets MotifPattern to originalMotif rotated by (rampCount % 3).
         */
        public static synchronized void shiftMotifForRamp() {
            if (originalMotif == null) {
                originalMotif = new EnumConstants.BallColor[]{
                    MotifPattern.getBallColorInSlotX(0),
                    MotifPattern.getBallColorInSlotX(1),
                    MotifPattern.getBallColorInSlotX(2)
                };
            }
            int shift = ballsInRamp % 3;
            MotifPattern.setBallPattern(
                originalMotif[shift],
                originalMotif[(shift + 1) % 3],
                originalMotif[(shift + 2) % 3]
            );
            lastShiftDebug = String.format("Ramp=%d shift=%d orig=[%s,%s,%s] -> [%s,%s,%s]",
                ballsInRamp, shift,
                originalMotif[0], originalMotif[1], originalMotif[2],
                MotifPattern.getBallColorInSlotX(0),
                MotifPattern.getBallColorInSlotX(1),
                MotifPattern.getBallColorInSlotX(2));
        }
    }

    /**
     * Tracks the detected motif ball pattern from Limelight.
     *
     * All methods are static - use the class directly (MotifPattern.getBallColorInSlotX(0))
     */
    public static class MotifPattern {
        private static EnumConstants.BallColor[] ballPattern = {
            EnumConstants.BallColor.None,
            EnumConstants.BallColor.None,
            EnumConstants.BallColor.None
        };

        /** Gets the ball color in the specified slot (0-2) */
        public static synchronized EnumConstants.BallColor getBallColorInSlotX(int x) {
            return ballPattern[x];
        }

        /** Sets all three slot colors at once */
        public static synchronized void setBallPattern(EnumConstants.BallColor zero, EnumConstants.BallColor one, EnumConstants.BallColor two) {
            ballPattern = new EnumConstants.BallColor[]{zero, one, two};
        }
    }
}
