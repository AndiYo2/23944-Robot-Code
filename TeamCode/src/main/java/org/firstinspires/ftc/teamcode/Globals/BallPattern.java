package org.firstinspires.ftc.teamcode.Globals;

public class BallPattern {

        private String[] pattern;

        public BallPattern(){
            pattern = new String[]{"ERROR","ERROR","ERROR"};
        }
        public BallPattern(String one, String two, String three){
            pattern = new String[]{one, two, three};
        }







        public void setBallPattern(String one, String two, String three){
            pattern = new String[]{one, two, three};
        }

        public String getBallInSlotX(int x){
            return pattern[x - 1];
        }

        public String[] getBallPattern(){
            return pattern;
        }
}
