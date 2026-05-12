package utility;

import Constants.EnumConstants;

public class SpindexerAndMotifStatus {

    public static class SpindexerPattern {
        private static EnumConstants.BallColor[] spindexerPattern = {
            EnumConstants.BallColor.None,
            EnumConstants.BallColor.None,
            EnumConstants.BallColor.None
        };

        public static synchronized EnumConstants.BallColor getBallInSlotX(int x) {
            return spindexerPattern[x];
        }

        public static synchronized void setBallInSlotX(int x, EnumConstants.BallColor ballType) {
            spindexerPattern[x] = ballType;
        }

        public static synchronized void setBallPattern(EnumConstants.BallColor zero, EnumConstants.BallColor one, EnumConstants.BallColor two) {
            spindexerPattern = new EnumConstants.BallColor[]{zero, one, two};
        }

        public static synchronized void clearAll() {
            setBallPattern(EnumConstants.BallColor.None, EnumConstants.BallColor.None, EnumConstants.BallColor.None);
        }

        public static synchronized void rotateBallsCCW() {
            EnumConstants.BallColor temp0 = spindexerPattern[0];
            spindexerPattern[0] = spindexerPattern[2];
            spindexerPattern[2] = spindexerPattern[1];
            spindexerPattern[1] = temp0;
        }

        public static synchronized void rotateBallsCW() {
            EnumConstants.BallColor temp0 = spindexerPattern[0];
            spindexerPattern[0] = spindexerPattern[1];
            spindexerPattern[1] = spindexerPattern[2];
            spindexerPattern[2] = temp0;
        }

        public static String getSpindexerPatternString() {
            return String.format("[I:%s S:%s T:%s]",
                    getBallInSlotX(0),  // I = Intake
                    getBallInSlotX(1),  // S = Shooter
                    getBallInSlotX(2)); // T = Top storage
        }

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

    public static class MotifPattern {
        private static EnumConstants.BallColor[] ballPattern = {
            EnumConstants.BallColor.None,
            EnumConstants.BallColor.None,
            EnumConstants.BallColor.None
        };

        public static synchronized EnumConstants.BallColor getBallColorInSlotX(int x) {
            return ballPattern[x];
        }

        public static synchronized void setBallPattern(EnumConstants.BallColor zero, EnumConstants.BallColor one, EnumConstants.BallColor two) {
            ballPattern = new EnumConstants.BallColor[]{zero, one, two};
        }
    }
}
