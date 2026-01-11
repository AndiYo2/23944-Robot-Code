package utility;

import Constants.EnumConstants;

public class SpindexerAndMotifStatus {

    public static class SpindexerPattern{
        // ZERO - Intake
        // ONE - Outtake
        // TWO - Top Slot

        private static EnumConstants.BallColor[] spindexerPattern;
        public SpindexerPattern(EnumConstants.BallColor zero, EnumConstants.BallColor one, EnumConstants.BallColor two){
            this.spindexerPattern = new EnumConstants.BallColor[]{zero, one, two};
        }
        public static EnumConstants.BallColor getBallInSlotX(int x){
            return spindexerPattern[x];
        }
        public void setBallPatternNone(int x){
            spindexerPattern[x] = EnumConstants.BallColor.None;
        }

        public static void setBallInSlotX(int x, EnumConstants.BallColor ballType){
            spindexerPattern[x] = ballType;
        }

        public static void setBallPattern(EnumConstants.BallColor zero, EnumConstants.BallColor one, EnumConstants.BallColor two){
            spindexerPattern = new EnumConstants.BallColor[]{zero, one, two};
        }

        public static void rotateBallsCCW(){
            EnumConstants.BallColor temp0 = spindexerPattern[0];
            spindexerPattern[0] = spindexerPattern[2];
            spindexerPattern[2] = spindexerPattern[1];
            spindexerPattern[1] = temp0;
        }

        public static void rotateBallsCW(){
            EnumConstants.BallColor temp0 = spindexerPattern[0];
            spindexerPattern[0] = spindexerPattern[1];
            spindexerPattern[1] = spindexerPattern[2];
            spindexerPattern[2] = temp0;
        }

        /**
         * Checks if all three spindexer slots contain balls.
         *
         * @return true if all slots are full, false otherwise
         */
        public static boolean isFull() {
            return spindexerPattern[0] != EnumConstants.BallColor.None
                    && spindexerPattern[1] != EnumConstants.BallColor.None
                    && spindexerPattern[2] != EnumConstants.BallColor.None;

        }


        public static String getSpindexerPatternString() {
            return String.format("[I:%s S:%s T:%s]",
                    getBallInSlotX(0),  // I = Intake
                    getBallInSlotX(1),  // S = Shooter
                    getBallInSlotX(2)); // T = sTorage
        }


    }


    public static class MotifPattern{
        private static EnumConstants.BallColor[] ballPattern;
        public MotifPattern(EnumConstants.BallColor zero, EnumConstants.BallColor one, EnumConstants.BallColor two){
            ballPattern = new EnumConstants.BallColor[]{zero, one, two};
        }
        public static EnumConstants.BallColor getBallColorInSlotX(int x){
            return ballPattern[x];
        }
        public static void setBallPattern(EnumConstants.BallColor[] pattern){
            ballPattern = pattern;
        }
        public static void setBallPattern(EnumConstants.BallColor zero, EnumConstants.BallColor one, EnumConstants.BallColor two){
            ballPattern = new EnumConstants.BallColor[]{zero, one, two};
        }
    }





}
