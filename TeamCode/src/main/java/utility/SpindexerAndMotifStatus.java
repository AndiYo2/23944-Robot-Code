package utility;

import static utility.RobotConstants.Spindexer.spindexerPattern;

public class SpindexerAndMotifStatus {

    public static class SpindexerPattern{
        // ZERO - Intake
        // ONE - Outtake
        // TWO - Top Slot

        private static RobotConstants.Enums.BallColor[] spindexerPattern;
        public SpindexerPattern(RobotConstants.Enums.BallColor zero, RobotConstants.Enums.BallColor one, RobotConstants.Enums.BallColor two){
            this.spindexerPattern = new RobotConstants.Enums.BallColor[]{zero, one, two};
        }
        public static RobotConstants.Enums.BallColor getBallInSlotX(int x){
            return spindexerPattern[x];
        }
        public void setBallPatternNone(int x){
            spindexerPattern[x] = RobotConstants.Enums.BallColor.None;
        }

        public static void setBallInSlotX(int x, RobotConstants.Enums.BallColor ballType){
            spindexerPattern[x] = ballType;
        }

        public static void setBallPattern(RobotConstants.Enums.BallColor zero, RobotConstants.Enums.BallColor one, RobotConstants.Enums.BallColor two){
            spindexerPattern = new RobotConstants.Enums.BallColor[]{zero, one, two};
        }

        public static void rotateBallsCW(){
            RobotConstants.Enums.BallColor temp0 = spindexerPattern[0];
            spindexerPattern[0] = spindexerPattern[1];
            spindexerPattern[1] = spindexerPattern[2];
            spindexerPattern[2] = temp0;
        }

        public static void rotateBallsCCW(){
            RobotConstants.Enums.BallColor temp0 = spindexerPattern[2];
            spindexerPattern[2] = spindexerPattern[1];
            spindexerPattern[1] = spindexerPattern[0];
            spindexerPattern[0] = temp0;
        }

        /**
         * Checks if all three spindexer slots contain balls.
         *
         * @return true if all slots are full, false otherwise
         */
        public static boolean isFull() {
            return spindexerPattern[0] != RobotConstants.Enums.BallColor.None
                    && spindexerPattern[1] != RobotConstants.Enums.BallColor.None
                    && spindexerPattern[2] != RobotConstants.Enums.BallColor.None;

        }

        public static String getSpindexerPatternString() {
            return String.format("[I:%s S:%s T:%s]",
                    getBallInSlotX(0),  // I = Intake
                    getBallInSlotX(1),  // S = Shooter
                    getBallInSlotX(2)); // T = sTorage
        }


    }


    public static class MotifPattern{
        private RobotConstants.Enums.BallColor[] ballPattern;
        public MotifPattern(RobotConstants.Enums.BallColor zero, RobotConstants.Enums.BallColor one, RobotConstants.Enums.BallColor two){
            this.ballPattern = new RobotConstants.Enums.BallColor[]{zero, one, two};
        }
        public RobotConstants.Enums.BallColor getBallColorInSlotX(int x){
            return this.ballPattern[x];
        }
        public void setBallPattern(RobotConstants.Enums.BallColor[] pattern){
            this.ballPattern = pattern;
        }
        public void setBallPattern(RobotConstants.Enums.BallColor zero, RobotConstants.Enums.BallColor one, RobotConstants.Enums.BallColor two){
            this.ballPattern = new RobotConstants.Enums.BallColor[]{zero, one, two};
        }
    }





}
